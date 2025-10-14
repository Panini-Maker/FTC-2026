package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.SimpleDriveActions;

@Autonomous
public class OutreachDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        SimpleDriveActions drive = new SimpleDriveActions(frontLeftMotor, frontRightMotor,
                backRightMotor, backLeftMotor, telemetry);

        for (int i = 0; i < 16; i++) {
            drive.drive(0.2, 0, 0, 1750);
            sleep(500);
            drive.drive(0, 0, -0.2, 2400);
            sleep(500);
        }
    }
}
