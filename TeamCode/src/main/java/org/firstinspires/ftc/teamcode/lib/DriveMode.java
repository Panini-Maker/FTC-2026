package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveMode {
    protected DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    protected DcMotorEx leftExtension, rightExtension, armMotor;
    protected Rev2mDistanceSensor leftDistance, frontDistance;
    protected BHI260IMU imu;
    protected ElapsedTime runtime;
    public ElapsedTime runtime2 = new ElapsedTime();
    protected Telemetry telemetry;

    public double forward;
    public double right;
    public double rotate;
    public long timeouts_ms; //timeout in milliseconds

    public Servo clawServo;

    public void init(DcMotor frontLeftMotor, DcMotor frontRightMotor,
                     DcMotor backRightMotor, DcMotor backLeftMotor,
                     DcMotorEx leftExtension, DcMotorEx rightExtension,
                     DcMotorEx armMotor, Servo clawServo, Rev2mDistanceSensor leftDistance,
                     Rev2mDistanceSensor frontDistance, ElapsedTime runtime,
                     Telemetry telemetry) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.leftExtension = leftExtension;
        this.rightExtension = rightExtension;
        this.armMotor = armMotor;
        this.clawServo = clawServo;
        this.leftDistance = leftDistance;
        this.frontDistance = frontDistance;
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public void encoderDrive() {
        if (runtime == null || frontLeftMotor == null || frontRightMotor == null ||
                backLeftMotor == null || backRightMotor == null) {
            throw new IllegalStateException("Motors or runtime are not initialized.");
        }
        runtime2.reset();
        // Set motor powers
        //Multipliers account for back right wheel offset
        frontRightMotor.setPower(forward + right + rotate);
        frontLeftMotor.setPower(forward - right - rotate);
        backLeftMotor.setPower(forward + right - rotate);
        backRightMotor.setPower(forward - right + rotate);
        /*telemetry.clear();
        telemetry.addData("runtime", runtime);

        telemetry.addData("Forward value: ", forward);
        telemetry.addData("Right value: ", right);
        telemetry.addData("Rotate value: ", rotate);
        telemetry.addData("Front right", (frontRightMotor.getPower()));
        telemetry.addData("Back right", (backRightMotor.getPower()));
        telemetry.addData("Front left", (frontLeftMotor.getPower()));
        telemetry.addData("Back left", (backLeftMotor.getPower()));
        telemetry.addData("FL Busy", frontLeftMotor.isBusy());
        telemetry.addData("FR Busy", frontRightMotor.isBusy());
        telemetry.addData("BL Busy", backLeftMotor.isBusy());
        telemetry.addData("BR Busy", backRightMotor.isBusy());
        telemetry.update();*/

        // Wait for the specified timeout or until the robot reaches its target
        if (timeouts_ms > 0) {
            while (runtime2.milliseconds() < timeouts_ms) {
                //Optionally, check if encoder position is reached
                if (runtime2.milliseconds() >= timeouts_ms) { //add or condition for encoder position
                    stopMotor2();
                    break;
                }
            }
        }
    }

    // Stop all motors
    public void stopMotor2() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
}