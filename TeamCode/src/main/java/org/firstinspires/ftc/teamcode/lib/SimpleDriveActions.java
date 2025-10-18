package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class SimpleDriveActions {
    //declares hardware and variables
    public Telemetry telemetry;
    public DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

    public double forward;
    public double right;
    public double rotate;
    public long timeouts_ms; //timeout in milliseconds

    //initializes hardware and variables for the DriveActions class
    public SimpleDriveActions(DcMotorEx frontLeftMotor, DcMotorEx frontRightMotor,
                              DcMotorEx backRightMotor, DcMotorEx backLeftMotor,
                              Telemetry telemetry) {

        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.telemetry = telemetry;
    }

    //basic drive function, drives using power and time
    public void drive(double forward, double right, double rotate, long timeouts_ms) throws InterruptedException {
        this.forward = forward;
        this.right = right;
        this.rotate = rotate;
        this.timeouts_ms = timeouts_ms;
        double frontRightPower = forward + right + rotate;
        double frontLeftPower = forward - right - rotate;
        double backLeftPower = forward + right - rotate;
        double backRightPower = forward - right + rotate;
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)));
        // Change normalization only when maxPower > 1 to only need to do division when necessary.
        if (maxPower > 1) {
            frontRightPower /= maxPower;
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        frontRightMotor.setPower(frontRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        Thread.sleep(timeouts_ms);
        stopMotor();
    }

    // Stop all motors
    public void stopMotor() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
}