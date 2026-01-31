package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueGoalPosition;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odometryHeadingSign;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redGoalPosition;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretMaxPower;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretMinPower;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretPhysicalOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTolerance;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTicksPerDegree;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AutoAim class handles automatic turret aiming based on robot position and target goal.
 * Uses PID control to smoothly aim the turret while the robot is moving.
 */
public class AutoAim {
    private DcMotorEx turret;
    private Telemetry telemetry;

    // PID variables
    private double integral = 0;
    private double previousError = 0;
    private long previousTime = 0;

    // Target tracking
    private boolean isRed = true; // Default to red team
    private boolean enabled = false;

    // Thread for parallel execution
    private volatile boolean aimRunning = false;
    private volatile double currentRobotX = 0;
    private volatile double currentRobotY = 0;
    private volatile double currentRobotHeading = 0; // in degrees
    private volatile double currentTurretHeading = 0; // in degrees (relative to robot)
    private Thread aimThread;

    public AutoAim(DcMotorEx turret, Telemetry telemetry, boolean isRed) {
        this.turret = turret;
        this.telemetry = telemetry;
        this.isRed = isRed;
        this.previousTime = System.currentTimeMillis();
    }

    /**
     * Sets the team color to determine which goal to aim at.
     * @param isRed true for red team, false for blue team
     */
    public void setTeam(boolean isRed) {
        this.isRed = isRed;
    }

    /**
     * Calculates the angle from the robot to the target goal.
     * @param robotX Robot X position in inches (field coordinates, origin at center)
     * @param robotY Robot Y position in inches (field coordinates, origin at center)
     * @param robotHeading Robot heading in degrees from odometry
     * @return The angle the turret needs to point at (in turret encoder degrees)
     *         Turret encoder: 0 = facing back of robot, CCW is positive, CW is negative
     */
    public double calculateTargetAngle(double robotX, double robotY, double robotHeading) {
        // Get target position based on team
        Vector2d target = isRed ? redGoalPosition : blueGoalPosition;

        // Calculate vector from robot to target
        double dx = target.x - robotX;
        double dy = target.y - robotY;

        // Calculate absolute angle to target (in degrees, 0 = positive X axis)
        // atan2 returns angle where 0 = positive X axis, CCW is positive
        // Negate both dx and dy to correct for coordinate system differences
        double absoluteAngle = Math.toDegrees(Math.atan2(dy, dx));

        // Adjust robot heading for odometry convention
        // odometryHeadingSign: -1 if odometry uses CW positive, 1 if CCW positive
        double adjustedRobotHeading = robotHeading * odometryHeadingSign;

        // Convert to angle relative to robot heading
        // After adjustment, heading should be in atan2 convention (0 = +X, CCW positive)
        double relativeAngleToRobotFront = absoluteAngle - adjustedRobotHeading;

        // Turret encoder 0 = facing back of robot (180° from front)
        // So we need to offset by turretPhysicalOffset to convert from "relative to front" to "turret encoder"
        // If target is directly in front of robot (relativeAngleToRobotFront = 0),
        // turret needs to be at 180° (pointing forward from its perspective)
        double turretTargetAngle = relativeAngleToRobotFront + turretPhysicalOffset;

        // Normalize to -180 to 180
        while (turretTargetAngle > 180) turretTargetAngle -= 360;
        while (turretTargetAngle < -180) turretTargetAngle += 360;

        // Clamp to turret limits (CCW is positive, CW is negative)
        if (turretTargetAngle > turretLimitCCW) {
            turretTargetAngle = turretLimitCCW;
        } else if (turretTargetAngle < turretLimitCW) {
            turretTargetAngle = turretLimitCW;
        }

        return turretTargetAngle;
    }

    /**
     * Updates the robot position for auto aim calculations.
     * Call this frequently with odometry data.
     */
    public void updateRobotPosition(double x, double y, double heading, double turretHeading) {
        this.currentRobotX = x;
        this.currentRobotY = y;
        this.currentRobotHeading = heading;
        this.currentTurretHeading = turretHeading;
    }

    /**
     * Calculates PID output to move turret to target angle.
     * @param targetAngle The desired turret angle in degrees
     * @param currentAngle The current turret angle in degrees
     * @return Motor power (-1 to 1)
     */
    public double calculatePID(double targetAngle, double currentAngle) {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - previousTime) / 1000.0; // Convert to seconds

        if (dt <= 0) dt = 0.012; // Default to ~80Hz if time hasn't changed

        double error = targetAngle - currentAngle;

        // Normalize error to -180 to 180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Proportional term
        double P = turretKp * error;

        // Integral term (with anti-windup)
        integral += error * dt;
        integral = Math.max(-100, Math.min(100, integral)); // Clamp integral
        double I = turretKi * integral;

        // Derivative term
        double derivative = (error - previousError) / dt;
        double D = turretKd * derivative;

        // Calculate total output
        double output = P + I + D;

        // Apply minimum power to overcome friction (only if not within tolerance)
        if (Math.abs(error) > turretTolerance) {
            if (Math.abs(output) < turretMinPower) {
                output = Math.copySign(turretMinPower, output);
            }
        } else {
            output = 0; // Within tolerance, stop
        }

        // Clamp to max power
        output = Math.max(-turretMaxPower, Math.min(turretMaxPower, output));

        // Save for next iteration
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    /**
     * Resets the PID controller state.
     */
    public void resetPID() {
        integral = 0;
        previousError = 0;
        previousTime = System.currentTimeMillis();
    }

    /**
     * Single update call for auto aim. Call this in your main loop.
     * @param robotX Robot X position in inches
     * @param robotY Robot Y position in inches
     * @param robotHeading Robot heading in degrees
     * @param currentTurretAngle Current turret angle in degrees (from encoder)
     * @return Motor power to apply to turret
     */
    public double update(double robotX, double robotY, double robotHeading, double currentTurretAngle) {
        double targetAngle = calculateTargetAngle(robotX, robotY, robotHeading);
        return calculatePID(targetAngle, currentTurretAngle);
    }

    /**
     * Starts auto aim in a parallel thread.
     * The thread will continuously update turret position based on robot position.
     */
    public void startAutoAim() {
        if (aimRunning) return; // Already running

        resetPID();
        aimRunning = true;

        aimThread = new Thread(() -> {
            while (aimRunning && !Thread.currentThread().isInterrupted()) {
                try {
                    double targetAngle = calculateTargetAngle(currentRobotX, currentRobotY, currentRobotHeading);
                    double power = calculatePID(targetAngle, currentTurretHeading);

                    turret.setPower(power);

                    Thread.sleep(12); // ~80Hz update rate
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Handle motor disconnection gracefully
                    break;
                }
            }
            // Stop turret when thread exits
            try {
                turret.setPower(0);
            } catch (Exception e) {
                // Motor disconnected
            }
        });
        aimThread.setDaemon(true);
        aimThread.start();
    }

    /**
     * Stops the auto aim thread.
     */
    public void stopAutoAim() {
        aimRunning = false;
        if (aimThread != null) {
            aimThread.interrupt();
            try {
                aimThread.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        try {
            turret.setPower(0);
        } catch (Exception e) {
            // Motor disconnected
        }
        resetPID();
    }

    /**
     * Returns whether auto aim is currently running.
     */
    public boolean isRunning() {
        return aimRunning;
    }

    /**
     * Gets the current target angle for telemetry purposes.
     */
    public double getTargetAngle() {
        return calculateTargetAngle(currentRobotX, currentRobotY, currentRobotHeading);
    }

    /**
     * Gets the error between target and current turret angle.
     */
    public double getError() {
        double targetAngle = calculateTargetAngle(currentRobotX, currentRobotY, currentRobotHeading);
        double error = targetAngle - currentTurretHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    /**
     * Checks if turret is within tolerance of target.
     */
    public boolean isOnTarget() {
        return Math.abs(getError()) <= turretTolerance;
    }

    /**
     * Gets the current turret heading from encoder.
     */
    public double getCurrentTurretHeading() {
        try {
            return turret.getCurrentPosition() / turretTicksPerDegree;
        } catch (Exception e) {
            return 0;
        }
    }

    /**
     * Debug method to get all intermediate calculation values.
     * Returns: [absoluteAngle, adjustedHeading, relativeAngle, turretTarget]
     */
    public double[] getDebugValues() {
        Vector2d target = isRed ? redGoalPosition : blueGoalPosition;
        double dx = target.x - currentRobotX;
        double dy = target.y - currentRobotY;
        double absoluteAngle = Math.toDegrees(Math.atan2(dy, dx));
        double adjustedRobotHeading = currentRobotHeading * odometryHeadingSign;
        double relativeAngleToRobotFront = absoluteAngle - adjustedRobotHeading;
        double turretTargetAngle = relativeAngleToRobotFront + turretPhysicalOffset;

        // Normalize
        while (turretTargetAngle > 180) turretTargetAngle -= 360;
        while (turretTargetAngle < -180) turretTargetAngle += 360;

        return new double[] {absoluteAngle, adjustedRobotHeading, relativeAngleToRobotFront, turretTargetAngle};
    }

    /**
     * Debug method to get the stored robot position values.
     * Returns: [storedX, storedY, storedHeading]
     */
    public double[] getStoredPosition() {
        return new double[] {currentRobotX, currentRobotY, currentRobotHeading};
    }
}