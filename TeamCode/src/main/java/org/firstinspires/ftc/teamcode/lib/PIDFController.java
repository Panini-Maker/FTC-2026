package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PIDF Controller with feedforward support.
 * Extends standard PID with a feedforward term (Kf) that applies a constant
 * output proportional to the setpoint, useful for velocity control.
 */
public class PIDFController {

    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    private double integral;
    private double previousError;
    private ElapsedTime timer;
    private Telemetry telemetry;
    private boolean firstRun;

    // Integral windup limits (prevents integral from growing too large)
    private static final double MAX_INTEGRAL = 1.0;  // Clamp integral contribution

    /**
     * Constructor for PIDF Controller.
     *
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param Kf Feedforward gain
     * @param telemetry Telemetry for debugging
     */
    public PIDFController(double Kp, double Ki, double Kd, double Kf, Telemetry telemetry) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.telemetry = telemetry;
        this.integral = 0;
        this.previousError = 0;
        this.timer = new ElapsedTime();
        this.firstRun = true;
    }

    /**
     * Constructor for PID Controller (without feedforward, Kf defaults to 0).
     *
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param telemetry Telemetry for debugging
     */
    public PIDFController(double Kp, double Ki, double Kd, Telemetry telemetry) {
        this(Kp, Ki, Kd, 0, telemetry);
    }

    /**
     * Calculates the motor power adjustment based on the desired and current velocity.
     * Includes feedforward term for better velocity tracking.
     *
     * @param target The target velocity/position for the motor.
     * @param current The current velocity/position of the motor.
     * @return The calculated adjustment for the motor power.
     */
    public double calculate(double target, double current) {
        double error = target - current;
        double deltaTime = timer.seconds();
        timer.reset();

        // On first run, don't accumulate integral or calculate derivative
        // Just return feedforward + proportional to avoid spikes
        if (firstRun) {
            firstRun = false;
            previousError = error;
            // Return only feedforward + proportional on first iteration
            return (Kf * target) + (Kp * error);
        }

        // Prevent issues when deltaTime is very small or unreasonably large
        if (deltaTime < 0.001) {
            deltaTime = 0.01;
        } else if (deltaTime > 0.5) {
            // If more than 500ms passed, something went wrong - don't accumulate integral
            deltaTime = 0.01;
        }

        // Accumulate integral
        integral += error * deltaTime;

        // Clamp integral to prevent windup
        if (Ki != 0) {
            double maxIntegral = MAX_INTEGRAL / Math.abs(Ki);
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        }

        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        // Feedforward: directly scales the target (useful for velocity control)
        double feedforward = Kf * target;

        /*
        telemetry.addData("PIDF Current", current);
        telemetry.addData("PIDF Target", target);
        telemetry.addData("PIDF Error", error);
        telemetry.addData("PIDF Integral", integral);
        telemetry.addData("PIDF Feedforward", feedforward);
        telemetry.addData("PIDF Output", feedforward + (Kp * error) + (Ki * integral) + (Kd * derivative));
        telemetry.update();
        */

        return feedforward + (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    /**
     * Resets the integral, previous error, and first run flag.
     * Call this when starting a new control sequence.
     */
    public void reset() {
        this.integral = 0;
        this.previousError = 0;
        this.firstRun = true;
        this.timer.reset();
    }

    // Getters and setters for tuning PIDF values dynamically
    public double getKp() {
        return Kp;
    }

    public void setKp(double Kp) {
        this.Kp = Kp;
    }

    public double getKi() {
        return Ki;
    }

    public void setKi(double Ki) {
        this.Ki = Ki;
    }

    public double getKd() {
        return Kd;
    }

    public void setKd(double Kd) {
        this.Kd = Kd;
    }

    public double getKf() {
        return Kf;
    }

    public void setKf(double Kf) {
        this.Kf = Kf;
    }

    /**
     * Set all PIDF constants at once.
     */
    public void setPIDFConstants(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }
}

