package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ShooterController;

@TeleOp
public class ShooterRegression extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors and servos
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);

        Servo light = hardwareMap.get(Servo.class, "light");

        float shooterSpeed = 1500;

        int latchState = 0;

        double hoodState = 0.15;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            if (gamepad2.aWasPressed()) {
                shooterSpeed += 10;
            } else if (gamepad2.bWasPressed()) {
                shooterSpeed -= 10;
            } else if (gamepad2.xWasPressed()) {
                shooterSpeed += 50;
            } else if (gamepad2.yWasPressed()) {
                shooterSpeed -= 50;
            }

            if (gamepad2.right_bumper) {
                intakeMotor.setPower(1);
                latchState = 0;
            } else if (gamepad2.left_bumper) {
                intakeMotor.setPower(0.6);
                latchState = 0;
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad2.right_trigger > 0) {
                latchState = 1;
                shooter.runShooter(shooterSpeed);
            } else {
                shooter.resetPID();
                shooter.stopShooter();
            }

            if (gamepad2.dpadUpWasPressed()) {
                if (hoodState < 0.54) {
                    hoodState += 0.03;
                }
            } else if (gamepad2.dpadDownWasPressed()) {
                if (hoodState > 0.15) {
                    hoodState -= 0.03;
                }
            }

            double shooterAverageSpeed = (rightShooter.getVelocity()+leftShooter.getVelocity()) / 2;

            if (Math.abs(shooterSpeed-shooterAverageSpeed) < 30) {
                light.setPosition(0.5);
            } else {
                light.setPosition(0.28);
            }

            rightLatch.setPosition(latchState);

            hood.setPosition(hoodState);

            telemetry.addData("Shooter Speed Target", shooterSpeed);
            telemetry.addData("Shooter Speed", (rightShooter.getVelocity()+leftShooter.getVelocity())/2);
            telemetry.addData("Hood Position", hoodState);
            telemetry.update();

        }
    }

}
