package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp, ki, kd;
    private double previousError = 0;
    private double integral = 0;
    private ElapsedTime timer;
    public PIDController(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.timer = new ElapsedTime();
    }

    public double calculate(double error){
        double dt = timer.seconds();
        timer.reset();

        integral += error * dt;
        double derivative = (error - previousError) / dt;
        previousError = error;
        return kp * error + ki * integral + kd * derivative;
    }

    public void reset(){
        previousError = 0;
        integral = 0;
        timer.reset();
    }
}

