package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKv;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTicksPerDegree;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {
    private DcMotorEx turret;
    // Parallel thread variables
    private volatile boolean pidRunning = false;
    private volatile double targetPosition = 0;
    private volatile double robotAngularVelocity = 0; // Robot's angular velocity in degrees/second
    private Thread pidThread;
    private Telemetry telemetry;

    // Encoder offset for error correction (in degrees)
    // This value is SUBTRACTED from the raw encoder reading to get the corrected heading
    private double encoderOffsetDegrees = 0;

    // Persistent PID controller (not recreated each loop)
    private PIDController pidController;

    private final double ticksPerDegree = turretTicksPerDegree;

    /**
     * Creates a Turret with initial heading of 0 degrees.
     * Encoder is reset at initialization.
     */
    public Turret(DcMotorEx turret, Telemetry telemetry) {
        this(turret, telemetry, 0, true);
    }

    /**
     * Creates a Turret with a specified initial heading.
     * Use this when starting TeleOp after autonomous to preserve turret position.
     *
     * @param turret The turret motor
     * @param telemetry Telemetry for debugging
     * @param initialHeadingDegrees The known heading of the turret at startup
     */
    public Turret(DcMotorEx turret, Telemetry telemetry, double initialHeadingDegrees) {
        this(turret, telemetry, initialHeadingDegrees, false);
    }

    /**
     * Creates a Turret with full control over initialization.
     *
     * @param turret The turret motor
     * @param telemetry Telemetry for debugging
     * @param initialHeadingDegrees The known heading of the turret at startup
     * @param resetEncoder Whether to reset the encoder (true for autonomous, false for teleop)
     */
    public Turret(DcMotorEx turret, Telemetry telemetry, double initialHeadingDegrees, boolean resetEncoder) {
        this.turret = turret;
        this.telemetry = telemetry;

        if (resetEncoder) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder at initialization
        }
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Set to use encoder

        // Calibrate to the initial heading
        calibrateCurrentPosition(initialHeadingDegrees);

        // Initialize PID controller
        this.pidController = new PIDController(turretKp, turretKi, turretKd, telemetry);
    }

    /**
     * Updates the robot's angular velocity for feedforward compensation.
     * Call this every loop with the current robot heading velocity.
     * @param angularVelocityDegPerSec Robot's angular velocity in degrees/second (positive = CCW)
     */
    public void setRobotAngularVelocity(double angularVelocityDegPerSec) {
        this.robotAngularVelocity = angularVelocityDegPerSec;
    }

    public void spinToPosition (int position, double power) {
        turret.setTargetPosition(position);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }

    public double spinToHeading (double headingDegrees, double power) {
        // Apply encoder offset to get corrected current position
        double currentHeadingDegrees = (turret.getCurrentPosition() / ticksPerDegree) - encoderOffsetDegrees;
        double error = headingDegrees - currentHeadingDegrees;

        double minPower = 0; // Minimum power to overcome friction
        double tolerance = 0.5; // 0.5 degree tolerance

        // Update PID constants in case they changed via dashboard
        pidController.setKp(turretKp);
        pidController.setKi(turretKi);
        pidController.setKd(turretKd);

        if (Math.abs(error) > tolerance) {
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // Calculate PID output using corrected heading
            double pidOutput = pidController.calculate(headingDegrees, currentHeadingDegrees);

            // Calculate velocity feedforward
            // When robot rotates CCW (positive angular velocity), turret needs to rotate CW (negative) to compensate
            // So we negate the angular velocity and multiply by kV
            //double velocityFeedforward = -turretKv * robotAngularVelocity;

            // Combine PID output with velocity feedforward
            double calculatedPower = pidOutput;// + velocityFeedforward;

            // Ensure minimum power to overcome friction, preserve direction
            calculatedPower = Math.copySign(Math.max(Math.abs(calculatedPower), minPower), calculatedPower);
            // Clamp to max power
            calculatedPower = Math.copySign(Math.min(Math.abs(calculatedPower), power), calculatedPower);
            turret.setPower(calculatedPower);
        } else {
            /*
            // Even at target, apply feedforward to counter robot rotation
            if (Math.abs(robotAngularVelocity) > 5) { // Only apply if rotating faster than 5 deg/sec
                double velocityFeedforward = -turretKv * robotAngularVelocity;
                velocityFeedforward = Math.copySign(Math.min(Math.abs(velocityFeedforward), power), velocityFeedforward);
                turret.setPower(velocityFeedforward);
            } else {
                stopTurret();
            }

             */
        }

        return currentHeadingDegrees;
    }

    public void spinToHeadingLoop (double desiredHeading, double power) {
        // If already running, just update target velocity
        if (pidRunning) {
            targetPosition = desiredHeading;
            return;
        }

        targetPosition = desiredHeading;
        pidRunning = true;
        pidController.reset(); // Reset PID state for new movement

        pidThread = new Thread(() -> {
            while (pidRunning && !Thread.currentThread().isInterrupted()) {
                spinToHeading(targetPosition, power);

                try {
                    Thread.sleep(10); // ~80Hz update rate
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
            // Ensure motors are stopped when thread exits
            turret.setPower(0);
        });
        pidThread.setDaemon(true); // Thread will stop when main program ends
        pidThread.start();
    }

    /**
     * Stops the parallel PID loop and the shooter motors.
     */
    public void stopVelocityPID() {
        pidRunning = false;
        // Immediately stop the turret motors
        turret.setPower(0);
        // Interrupt the PID thread to exit quickly
        if (pidThread != null) {
            pidThread.interrupt();
            // Don't join them
            pidThread = null;
        }
    }

    /**
     * Updates the PID constants for live tuning via FTC Dashboard.
     */
    public void updatePIDConstants(double kp, double ki, double kd) {
        pidController.setKp(kp);
        pidController.setKi(ki);
        pidController.setKd(kd);
    }


    /**
     * Blocking version of spinToHeading for autonomous.
     * Waits until the turret reaches the target position before returning.
     */
    public double spinToHeadingBlocking(double headingDegrees, double power, long timeoutMs) {
        double tolerance = 2.0; // 2 degree tolerance
        double slowDownZone = 10.0; // 10 degrees from target
        double kP = 0.01;
        double minPower = 0.15;

        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < timeoutMs) {
            // Apply encoder offset to get corrected current position
            double currentHeadingDegrees = (turret.getCurrentPosition() / ticksPerDegree) - encoderOffsetDegrees;
            double error = headingDegrees - currentHeadingDegrees;

            // Check if within tolerance
            if (Math.abs(error) <= tolerance) {
                stopTurret();
                return currentHeadingDegrees;
            }

            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            double calculatedPower = kP * error * ticksPerDegree; // Scale error to ticks for power calculation

            // Only apply minimum power if far from target
            if (Math.abs(error) > slowDownZone) {
                calculatedPower = Math.copySign(Math.max(Math.abs(calculatedPower), minPower), calculatedPower);
            }

            // Clamp to max power
            calculatedPower = Math.copySign(Math.min(Math.abs(calculatedPower), power), calculatedPower);
            turret.setPower(calculatedPower);

            // Small delay to prevent busy waiting
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // Timeout reached, stop turret
        stopTurret();
        return (turret.getCurrentPosition() / ticksPerDegree) - encoderOffsetDegrees;
    }

    public boolean isWithinTolerance(double headingDegrees) {
        double currentHeadingDegrees = (turret.getCurrentPosition() / ticksPerDegree) - encoderOffsetDegrees;
        double error = headingDegrees - currentHeadingDegrees;
        double tolerance = 1.0; // 1 degree tolerance
        return Math.abs(error) <= tolerance;
    }

    public double getErrorDegrees(double headingDegrees) {
        double currentHeadingDegrees = (turret.getCurrentPosition() / ticksPerDegree) - encoderOffsetDegrees;
        return headingDegrees - currentHeadingDegrees;
    }

    public void stopTurret() {
        turret.setPower(0);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Gets the current turret heading in degrees, with encoder offset applied.
     * @return Current heading in degrees (corrected for any offset)
     */
    public double getCurrentHeading() {
        return (turret.getCurrentPosition() / ticksPerDegree) - encoderOffsetDegrees;
    }

    /**
     * Gets the raw encoder heading without any offset applied.
     * @return Raw encoder heading in degrees
     */
    public double getRawHeading() {
        return turret.getCurrentPosition() / ticksPerDegree;
    }

    /**
     * Sets the encoder offset to correct for turret position errors.
     * Use this when the turret's actual position doesn't match where it thinks it is.
     *
     * Example: If turret thinks it's at 0 degrees but is actually at 5 degrees,
     * call setEncoderOffset(5) to correct. After this, getCurrentHeading() will
     * return the corrected value.
     *
     * @param offsetDegrees The offset in degrees (actual position - reported position)
     */
    public void setEncoderOffset(double offsetDegrees) {
        this.encoderOffsetDegrees = offsetDegrees;
    }

    /**
     * Calibrates the turret by setting the current position as a known heading.
     * This calculates and applies the appropriate encoder offset.
     *
     * Example: If turret is physically at 5 degrees but encoder reads 0,
     * call calibrateCurrentPosition(5) to set the offset so getCurrentHeading() returns 5.
     *
     * @param actualHeadingDegrees The actual/known heading of the turret in degrees
     */
    public void calibrateCurrentPosition(double actualHeadingDegrees) {
        double rawHeading = turret.getCurrentPosition() / ticksPerDegree;
        this.encoderOffsetDegrees = rawHeading - actualHeadingDegrees;
    }

    /**
     * Gets the current encoder offset.
     * @return The encoder offset in degrees
     */
    public double getEncoderOffset() {
        return encoderOffsetDegrees;
    }

    /**
     * Resets the encoder offset to zero.
     */
    public void resetEncoderOffset() {
        this.encoderOffsetDegrees = 0;
    }

    public double turnToGoal(Pose2D pos, double turretCurrentAngle, boolean targetIsRed) {
        // Get robot x, y, heading
        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double heading = pos.getHeading(AngleUnit.DEGREES);

        // find goal coordinates
        double goal_x;
        double goal_y;

        if (targetIsRed) {
            goal_x = 144;
            goal_y = 144;
        } else {
            goal_x = 0;
            goal_y = 144;
        }

        double dist_x = x - goal_x;
        double dist_y = y - goal_y;

        double fieldAngle = Math.toDegrees(Math.atan(dist_x/dist_y));
        double robotAngle = fieldAngle - heading;
        double turretAngle = robotAngle - turretCurrentAngle;

        return turretAngle;
    }

}
