package org.firstinspires.ftc.teamcode.lib;

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
    private final PIDFController pidfController;
    private final Telemetry telemetry;
    private final VoltageSensor voltageSensor;

    // Nominal voltage for Kf compensation
    private static final double NOMINAL_VOLTAGE = 12.5;

    // Base Kf value (before voltage compensation)
    private final double baseKf;

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
        this(leftShooter, rightShooter, Kp, Ki, Kd, 0, null, telemetry);
    }

    /**
     * Constructor for ShooterController using PIDF (with feedforward).
     * @deprecated Use constructor with VoltageSensor for voltage compensation
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, double Kf, Telemetry telemetry) {
        this(leftShooter, rightShooter, Kp, Ki, Kd, Kf, null, telemetry);
    }

    /**
     * Constructor for ShooterController using PIDF with voltage compensation.
     *
     * @param voltageSensor VoltageSensor for voltage compensation (can be null to disable)
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, double Kf, VoltageSensor voltageSensor, Telemetry telemetry) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;
        this.baseKf = Kf;

        // Initialize PIDF controller
        this.pidfController = new PIDFController(Kp, Ki, Kd, Kf, telemetry);

        // Set motor directions - rightShooter is reversed so both spin the flywheel the same way
        this.leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Runs the shooter motors at the desired velocity using PIDF control.
     *
     * @param desiredVelocity The target velocity in ticks per second.
     */
    public void runShooter(double desiredVelocity) {
        // Apply voltage compensation to Kf
        // Lower voltage = need more power = higher Kf
        double compensatedKf = baseKf;
        if (voltageSensor != null) {
            double currentVoltage = voltageSensor.getVoltage();
            debugVoltage = currentVoltage;
            // Prevent division by very low voltage
            if (currentVoltage > 8.0) {
                compensatedKf = baseKf * (NOMINAL_VOLTAGE / currentVoltage);
                pidfController.setKf(compensatedKf);
            }
        }
        debugCompensatedKf = compensatedKf;

        // Use Math.abs() because rightShooter has reversed direction, so getVelocity() returns negative
        double leftVel = leftShooter.getVelocity();
        double rightVel = rightShooter.getVelocity();
        double currentVelocity = (Math.abs(leftVel) + Math.abs(rightVel)) / 2.0;
        double powerAdjustment = pidfController.calculate(desiredVelocity, currentVelocity);

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
        return String.format("L=%.0f R=%.0f Avg=%.0f Pwr=%.2f V=%.1f Kf=%.6f",
                debugLeftVel, debugRightVel, debugAvgVel, debugPower, debugVoltage, debugCompensatedKf);
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

        pidfThread = new Thread(() -> {
            while (pidfRunning && !Thread.currentThread().isInterrupted()) {
                runShooter(targetVelocity);

                try {
                    Thread.sleep(12); // ~80Hz update rate
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
        if (pidfThread != null) {
            pidfThread.interrupt();
            try {
                pidfThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        stopShooter();
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
