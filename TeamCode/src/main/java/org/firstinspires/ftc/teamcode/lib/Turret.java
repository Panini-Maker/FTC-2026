package org.firstinspires.ftc.teamcode.lib;

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
    private Thread pidThread;
    private Telemetry telemetry;

    private final double ticksPerDegree = turretTicksPerDegree;
    public Turret(DcMotorEx turret, Telemetry telemetry) {
        this.turret = turret;
        this.telemetry = telemetry;
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder at initialization
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Set to use encoder
    }

    public void spinToPosition (int position, double power) {
        turret.setTargetPosition(position);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }

    public double spinToHeading (double headingDegrees, double power) {
        int targetPosition = (int)(headingDegrees * ticksPerDegree);
        int currentPosition = turret.getCurrentPosition();
        int error = targetPosition - currentPosition;

        double kP = 0.005; // Proportional constant, tune as needed
        double minPower = 0.15; // Minimum power to overcome friction
        int tolerance = (int)(ticksPerDegree); // 1 degree tolerance

        if (Math.abs(error) > tolerance) {
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            double calculatedPower = kP * error;
            // Ensure minimum power to overcome friction, preserve direction
            calculatedPower = Math.copySign(Math.max(Math.abs(calculatedPower), minPower), calculatedPower);
            // Clamp to max power
            calculatedPower = Math.copySign(Math.min(Math.abs(calculatedPower), power), calculatedPower);
            turret.setPower(calculatedPower);
        } else {
            stopTurret();
        }

        // Debugging telemetry
        /*
        telemetry.addLine("Target Position (ticks): " + targetPosition);
        telemetry.addLine("Current Position (ticks): " + currentPosition);
        telemetry.addLine("Error (ticks): " + error);
        telemetry.addLine("Ticks per Degree: " + ticksPerDegree);

         */

        return (currentPosition / ticksPerDegree);
    }

    public void spinToHeadingLoop (double desiredHeading, double power) {
        // If already running, just update target velocity
        if (pidRunning) {
            targetPosition = desiredHeading;
            return;
        }

        targetPosition = desiredHeading;
        pidRunning = true;

        pidThread = new Thread(() -> {
            while (pidRunning && !Thread.currentThread().isInterrupted()) {
                spinToHeading(targetPosition, power);

                try {
                    Thread.sleep(12); // ~80Hz update rate
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
        if (pidThread != null) {
            pidThread.interrupt();
            try {
                pidThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        stopTurret();
    }

    /**
     * Blocking version of spinToHeading for autonomous.
     * Waits until the turret reaches the target position before returning.
     */
    public double spinToHeadingBlocking(double headingDegrees, double power, long timeoutMs) {
        int targetPosition = (int)(headingDegrees * ticksPerDegree);
        int tolerance = (int)(2 * ticksPerDegree); // 2 degree tolerance
        int slowDownZone = (int)(10 * ticksPerDegree); // 10 degrees from target
        double kP = 0.01;
        double minPower = 0.15;

        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < timeoutMs) {
            int currentPosition = turret.getCurrentPosition();
            int error = targetPosition - currentPosition;

            // Check if within tolerance
            if (Math.abs(error) <= tolerance) {
                stopTurret();
                return currentPosition / ticksPerDegree;
            }

            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            double calculatedPower = kP * error;

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
        return turret.getCurrentPosition() / ticksPerDegree;
    }

    public boolean isWithinTolerance(double headingDegrees) {
        int targetPosition = (int)(headingDegrees * ticksPerDegree);
        int currentPosition = turret.getCurrentPosition();
        int error = targetPosition - currentPosition;
        int tolerance = (int)(ticksPerDegree); // 1 degree tolerance
        return Math.abs(error) <= tolerance;
    }

    public double getErrorDegrees(double headingDegrees) {
        int targetPosition = (int)(headingDegrees * ticksPerDegree);
        int currentPosition = turret.getCurrentPosition();
        int error = targetPosition - currentPosition;
        return error / ticksPerDegree;
    }

    public void stopTurret() {
        turret.setPower(0);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
