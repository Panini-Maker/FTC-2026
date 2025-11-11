package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

public class WheelPIDControl{

    private double Kp, Ki, Kd;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime;

    public WheelPIDControl(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.lastTime = System.nanoTime();
    }

    public double update(double target, double current){
        double currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;

        double error =  target - current;
        integral += error * dt;
        double derivative = (error-lastError) / dt;

        double output = Kp * error + Ki * integral + Kd * derivative;

        lastError = error;
        lastTime = currentTime;

        return output;
    }




}
