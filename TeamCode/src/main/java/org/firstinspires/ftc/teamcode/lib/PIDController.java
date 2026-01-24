package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {

    private double Kp;
    private double Ki;
    private double Kd;

    private double integral;
    private double previousError;
    private ElapsedTime timer;
    private Telemetry telemetry;

    public PIDController(double Kp, double Ki, double Kd, Telemetry telemetry) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.telemetry = telemetry;
        this.integral = 0;
        this.previousError = 0;
        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the motor power adjustment based on the desired and current velocity.
     *
     * @param desiredVelocity The target velocity for the motor.
     * @param currentVelocity The current velocity of the motor.
     * @return The calculated adjustment for the motor power.
     */
    public double calculate(double desiredVelocity, double currentVelocity) {
        double error = desiredVelocity - currentVelocity;
        double deltaTime = timer.seconds();
        timer.reset();

        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        /*
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Desired Velocity", desiredVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Previous Error", previousError);
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("PID Output", (Kp * error) + (Ki * integral) + (Kd * derivative));
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.update();

         */

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    /**
     * Resets the integral and previous error values.
     */
    public void reset() {
        this.integral = 0;
        this.previousError = 0;
        this.timer.reset();
    }

    // Getters and setters for tuning PID values dynamically
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
}
