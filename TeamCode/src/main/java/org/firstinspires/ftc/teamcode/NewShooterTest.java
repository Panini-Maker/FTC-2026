package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
--------------------------------------------------------------------------------------------------------------
ONLY WORKS WITH CORRECT SHOOTER CONFIGURATION
--------------------------------------------------------------------------------------------------------------
*/

@TeleOp
public class NewShooterTest extends LinearOpMode{

    @Override

    public void runOpMode() throws InterruptedException {
        // Map motors and servos
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo hood = hardwareMap.get(Servo.class, "hoodServo");

        // Light
        Servo light = hardwareMap.get(Servo.class, "light");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // variables

        long shooterSpeed;

        double intakeSpeed = 0.8;

        int latch = 0; // 0 = closed, 1 = open

        ElapsedTime cooldown = new ElapsedTime();

        double hoodAngle = 1; // 1 is all the way down, 0 is all the way up

        rightLatch.setPosition(1 - latch); // 1 is closed, 0 is open
        leftLatch.setPosition(latch); // 0 is closed, 1 is open
        hood.setPosition(hoodAngle);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            /// Control Settings

            // Shooter
            if (gamepad2.right_trigger > 0) {
                /// IMPORTANT: leftShooter must be set at negative, to reverse direction
                shooterSpeed = 1400;
            } else if (gamepad2.left_trigger > 0) {
                shooterSpeed = -300;
            } else {
                shooterSpeed = 0;
            }

            // Latch
            if (gamepad2.right_bumper && (cooldown.milliseconds() > 200)) {
                latch = 1 - latch;
                cooldown.reset();
            }

            // Hood (for testing only

            if (gamepad2.dpad_up) {
                if (hoodAngle > 0) {
                    hoodAngle -= 0.01;
                }
            } else if (gamepad2.dpad_down) {
                if (hoodAngle < 1) {
                    hoodAngle += 0.01;
                }
            }

            if (gamepad1.right_trigger > 0) {
                intakeMotor.setPower(intakeSpeed);
            } else if (gamepad1.left_trigger > 0) {
                intakeMotor.setPower(-intakeSpeed);
            } else {
                intakeMotor.setPower(0);
            }

            ///  Set Power to Motors/Servos

            rightShooter.setVelocity(shooterSpeed);
            leftShooter.setVelocity(shooterSpeed);

            rightLatch.setPosition(1 - latch);
            leftLatch.setPosition(latch);

            hood.setPosition(hoodAngle);

            double avgShooterVel = (leftShooter.getVelocity() + rightShooter.getVelocity())/2;

            if (avgShooterVel > 1380 && avgShooterVel < 1420) {
                light.setPosition(0.5);
            } else {
                light.setPosition(0.3);
            }

            telemetry.addData("Shooter Velocity", avgShooterVel);
            telemetry.addData("Shooter Angle", hoodAngle);
            telemetry.addData("Latch", latch);
            telemetry.update();

        }




        }
}
