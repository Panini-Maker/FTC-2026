package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterDualPIDThreshold;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ShooterController {

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final PIDFController pidfController;       // Primary PIDF (coarse control)
    private final PIDFController pidfController2;      // Secondary PIDF (fine control)
    private final Telemetry telemetry;
    private final VoltageSensor voltageSensor;

    // Nominal voltage for Kf compensation
    private static final double NOMINAL_VOLTAGE = 12.5;

    // Base Kf values (before voltage compensation)
    private final double baseKf;
    private final double baseKf2;


    // Track which controller is active for debugging
    private volatile boolean usingSecondaryPID = false;

    // Parallel thread variables
    private volatile boolean pidfRunning = false;
    private volatile double targetVelocity = 0;
    private Thread pidfThread;

    // Debug variables (accessible from main thread)
    private volatile double debugLeftVel = 0;
    private volatile double debugRightVel = 0;
    private volatile double debugAvgVel = 0;
    private volatile double debugPower = 0;
    private volatile double debugVoltage = 0;
    private volatile double debugCompensatedKf = 0;

    /**
     * Constructor for ShooterController using PID (without feedforward).
     * Maintains backwards compatibility with existing code using PID tuner.
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, Telemetry telemetry) {
        this(leftShooter, rightShooter, Kp, Ki, Kd, 0, Kp, Ki, Kd, 0, null, telemetry);
    }

    /**
     * Constructor for ShooterController using PIDF (with feedforward).
     * @deprecated Use constructor with VoltageSensor for voltage compensation
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, double Kf, Telemetry telemetry) {
        this(leftShooter, rightShooter, Kp, Ki, Kd, Kf, Kp, Ki, Kd, Kf, null, telemetry);
    }

    /**
     * Constructor for ShooterController using PIDF with voltage compensation.
     * Uses same PIDF values for both primary and secondary.
     *
     * @param voltageSensor VoltageSensor for voltage compensation (can be null to disable)
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, double Kf, VoltageSensor voltageSensor, Telemetry telemetry) {
        this(leftShooter, rightShooter, Kp, Ki, Kd, Kf, Kp, Ki, Kd, Kf, voltageSensor, telemetry);
    }

    /**
     * Constructor for ShooterController using dual PIDF with voltage compensation.
     * Primary PIDF for coarse control (far from target), Secondary PIDF for fine control (close to target).
     * Threshold is read from TuningVars.shooterDualPIDThreshold for live tuning via FTC Dashboard.
     *
     * @param Kp Primary proportional constant
     * @param Ki Primary integral constant
     * @param Kd Primary derivative constant
     * @param Kf Primary feedforward constant
     * @param Kp2 Secondary proportional constant (fine control)
     * @param Ki2 Secondary integral constant (fine control)
     * @param Kd2 Secondary derivative constant (fine control)
     * @param Kf2 Secondary feedforward constant (fine control)
     * @param voltageSensor VoltageSensor for voltage compensation (can be null to disable)
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter,
                            double Kp, double Ki, double Kd, double Kf,
                            double Kp2, double Ki2, double Kd2, double Kf2,
                            VoltageSensor voltageSensor, Telemetry telemetry) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;
        this.baseKf = Kf;
        this.baseKf2 = Kf2;

        // Initialize PIDF controllers
        this.pidfController = new PIDFController(Kp, Ki, Kd, Kf, telemetry);
        this.pidfController2 = new PIDFController(Kp2, Ki2, Kd2, Kf2, telemetry);

        // Set motor directions - rightShooter is reversed so both spin the flywheel the same way
        this.leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Runs the shooter motors at the desired velocity using dual PIDF control.
     * Uses primary PIDF when far from target, secondary PIDF when close to target.
     *
     * @param desiredVelocity The target velocity in ticks per second.
     */
    public void runShooter(double desiredVelocity) {
        // Use Math.abs() because rightShooter has reversed direction, so getVelocity() returns negative
        double leftVel = leftShooter.getVelocity();
        double rightVel = rightShooter.getVelocity();
        double currentVelocity = (Math.abs(leftVel) + Math.abs(rightVel)) / 2.0;
        double error = Math.abs(desiredVelocity - currentVelocity);

        // Select which controller to use based on error (threshold from TuningVars for live tuning)
        PIDFController activeController;
        double activeBaseKf;
        if (error <= shooterDualPIDThreshold) {
            activeController = pidfController2;
            activeBaseKf = baseKf2;
            usingSecondaryPID = true;
        } else {
            activeController = pidfController;
            activeBaseKf = baseKf;
            usingSecondaryPID = false;
        }

        // Apply voltage compensation to Kf
        // Lower voltage = need more power = higher Kf
        double compensatedKf = activeBaseKf;
        if (voltageSensor != null) {
            double currentVoltage = voltageSensor.getVoltage();
            debugVoltage = currentVoltage;
            // Prevent division by very low voltage
            if (currentVoltage > 8.0) {
                compensatedKf = activeBaseKf * (NOMINAL_VOLTAGE / currentVoltage);
                activeController.setKf(compensatedKf);
            }
        }
        debugCompensatedKf = compensatedKf;

        double powerAdjustment = activeController.calculate(desiredVelocity, currentVelocity);

        // Clamp power to valid range
        powerAdjustment = Math.max(-1.0, Math.min(1.0, powerAdjustment));

        leftShooter.setPower(powerAdjustment);
        rightShooter.setPower(powerAdjustment);

        // Store debug values
        debugLeftVel = leftVel;
        debugRightVel = rightVel;
        debugAvgVel = currentVelocity;
        debugPower = powerAdjustment;
    }

    /**
     * Gets debug info string for telemetry.
     */
    @SuppressLint("DefaultLocale")
    public String getDebugInfo() {
        String pidMode = usingSecondaryPID ? "FINE" : "COARSE";
        return String.format("L=%.0f R=%.0f Avg=%.0f Pwr=%.2f V=%.1f [%s]",
                debugLeftVel, debugRightVel, debugAvgVel, debugPower, debugVoltage, pidMode);
    }

    /**
     * Starts the shooter at the desired velocity using PIDF control in a parallel thread.
     * This replaces setVelocity and can run in parallel with other actions.
     *
     * @param desiredVelocity The target velocity in ticks per second.
     */
    public void setVelocityPIDF(double desiredVelocity) {
        // If already running, just update target velocity
        if (pidfRunning) {
            targetVelocity = desiredVelocity;
            return;
        }

        targetVelocity = desiredVelocity;
        pidfRunning = true;
        pidfController.reset();
        pidfController2.reset();

        pidfThread = new Thread(() -> {
            while (pidfRunning && !Thread.currentThread().isInterrupted()) {
                runShooter(targetVelocity);

                try {
                    Thread.sleep(10); // ~80Hz update rate
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
            // Ensure motors are stopped when thread exits
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        });
        pidfThread.setDaemon(true); // Thread will stop when main program ends
        pidfThread.start();
    }

    /**
     * Updates the target velocity while the PIDF loop is running.
     *
     * @param desiredVelocity The new target velocity in ticks per second.
     */
    public void updateVelocityPIDF(double desiredVelocity) {
        targetVelocity = desiredVelocity;
    }

    /**
     * Stops the parallel PIDF loop and the shooter motors.
     */
    public void stopVelocityPIDF() {
        pidfRunning = false;
        // Stop motors immediately
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        // Interrupt the thread to exit quickly
        if (pidfThread != null) {
            pidfThread.interrupt();
            // Don't join them
            pidfThread = null;
        }
    }

    /**
     * Returns whether the PIDF loop is currently running.
     */
    public boolean isRunning() {
        return pidfRunning;
    }

    // ==================== DEPRECATED METHODS (for backwards compatibility) ====================

    /**
     * @deprecated Use {@link #setVelocityPIDF(double)} instead.
     */
    @Deprecated
    public void setVelocityPID(double desiredVelocity) {
        setVelocityPIDF(desiredVelocity);
    }

    /**
     * @deprecated Use {@link #updateVelocityPIDF(double)} instead.
     */
    @Deprecated
    public void updateVelocityPID(double desiredVelocity) {
        updateVelocityPIDF(desiredVelocity);
    }

    /**
     * @deprecated Use {@link #stopVelocityPIDF()} instead.
     */
    @Deprecated
    public void stopVelocityPID() {
        stopVelocityPIDF();
    }

    /**
     * @deprecated Use {@link #resetPIDF()} instead.
     */
    @Deprecated
    public void resetPID() {
        resetPIDF();
    }

    // ==================== END DEPRECATED METHODS ====================

    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        pidfController.reset();
    }

    /**
     * Sets raw power to the shooter motors (bypasses PIDF control).
     * Use this for idle spinning when not actively shooting.
     *
     * @param power Raw power value from -1.0 to 1.0
     */
    public void setRawPower(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    /**
     * Resets the PIDF controller.
     */
    public void resetPIDF() {
        pidfController.reset();
    }

    /**
     * Dynamically update PID constants.
     */
    public void setPIDConstants(double Kp, double Ki, double Kd) {
        pidfController.setKp(Kp);
        pidfController.setKi(Ki);
        pidfController.setKd(Kd);
    }

    /**
     * Dynamically update PIDF constants (including feedforward).
     */
    public void setPIDFConstants(double Kp, double Ki, double Kd, double Kf) {
        pidfController.setPIDFConstants(Kp, Ki, Kd, Kf);
    }

    /**
     * Updates the PIDF constants for live tuning via FTC Dashboard.
     * Alias for setPIDFConstants.
     */
    public void updatePIDFConstants(double Kp, double Ki, double Kd, double Kf) {
        pidfController.setKp(Kp);
        pidfController.setKi(Ki);
        pidfController.setKd(Kd);
        pidfController.setKf(Kf);
    }

    public double getDistanceFromGoal(Pose2D pos, boolean targetIsRed) {

        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double goal_pos_x;
        double goal_pos_y;

        if (targetIsRed) {
            goal_pos_x = 144;
            goal_pos_y = 144;
        } else {
            goal_pos_x = 0;
            goal_pos_y = 144;
        }

        double distance;

        distance = Math.sqrt(Math.pow((goal_pos_x-x), 2) + Math.pow((goal_pos_y-y), 2));

        return distance;
    }

    public double getShooterRPM(double distance) {
        return distance * 7.05 + 1019;
    }

    public double getShooterAngle(double distance) {
        return 0.061      + 0.00602 * distance - 0.0000207 * distance * distance;
    }
}
