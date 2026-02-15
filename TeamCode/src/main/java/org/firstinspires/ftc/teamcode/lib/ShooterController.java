package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ShooterController {

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final PIDFController pidfController;
    private final Telemetry telemetry;

    // Parallel thread variables
    private volatile boolean pidfRunning = false;
    private volatile double targetVelocity = 0;
    private Thread pidfThread;

    /**
     * Constructor for ShooterController using PID (without feedforward).
     * Maintains backwards compatibility with existing code using PID tuner.
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, Telemetry telemetry) {
        this(leftShooter, rightShooter, Kp, Ki, Kd, 0, telemetry);
    }

    /**
     * Constructor for ShooterController using PIDF (with feedforward).
     */
    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, double Kf, Telemetry telemetry) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.telemetry = telemetry;

        // Initialize PIDF controller
        this.pidfController = new PIDFController(Kp, Ki, Kd, Kf, telemetry);

        // Set motor directions if needed
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
        double currentVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
        double powerAdjustment = pidfController.calculate(desiredVelocity, currentVelocity);

        leftShooter.setPower(powerAdjustment);
        rightShooter.setPower(powerAdjustment);

        //telemetry.addData("Shooter Power Adjustment", powerAdjustment);
        //telemetry.update();
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
