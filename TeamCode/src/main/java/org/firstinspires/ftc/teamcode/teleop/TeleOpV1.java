package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOpV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor loadingTransfer = hardwareMap.dcMotor.get("loading");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotor intakeTransfer = hardwareMap.dcMotor.get("intakeTransfer");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        //rightShooter.setDirection(DcMotor.Direction.REVERSE);

        double targetVelocity = 2100; // 4500 RPM in ticks/sec
        double kP = 0.001, kI = 0.0001, kD = 0.0001;
        double leftIntegral = 0, rightIntegral = 0;
        double leftLastError = 0, rightLastError = 0;

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            double leftCurrentVelocity = leftShooter.getVelocity();
            //double rightCurrentVelocity = rightShooter.getVelocity();

            double leftError = targetVelocity - leftCurrentVelocity;
            //double rightError = targetVelocity - rightCurrentVelocity;

            leftIntegral += leftError * timer.seconds();
            //rightIntegral += rightError * timer.seconds();

            double leftDerivative = (leftError - leftLastError) / timer.seconds();
            //double rightDerivative = (rightError - rightLastError) / timer.seconds();

            double leftOutput = kP * leftError + kI * leftIntegral + kD * leftDerivative;
            //double rightOutput = kP * rightError + kI * rightIntegral + kD * rightDerivative;

            if (gamepad2.right_trigger > 0) {
                leftShooter.setPower(0.8); // add leftOutput once tuned
                //rightShooter.setPower(rightOutput);
            } else if (gamepad2.left_trigger > 0) {
                leftShooter.setPower(-0.3); //for intaking human player balls

            } else {
                leftShooter.setPower(0);
                //rightShooter.setPower(0);
            }

            leftLastError = leftError;
            //rightLastError = rightError;
            timer.reset();

            telemetry.addData("Left Target Velocity", targetVelocity);
            telemetry.addData("Left Current Velocity", leftCurrentVelocity);
            telemetry.addData("Left Error", leftError);
            telemetry.addData("Right Target Velocity", targetVelocity);
            //telemetry.addData("Right Current Velocity", rightCurrentVelocity);
            //telemetry.addData("Right Error", rightError);
            telemetry.update();

            double multiplier = 0.8; //Used teo limit speed for testing/safety
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Turning

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) * multiplier / denominator;
            double backLeftPower = (y - x + rx) * multiplier / denominator;
            double frontRightPower = (y - x - rx) * multiplier / denominator;
            double backRightPower = (y + x - rx) * multiplier / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Intake and Transfer Controls
            // right trigger to shooting balls
            //left trigger for intaking and transferring balls
            // if both triggers are pressed, the robot will do both actions simultaneously
            // right button is the outake in case we intake too many artifacts

            if (gamepad2.right_bumper) {
                intakeTransfer.setPower(0.8);
            } else if (gamepad2.left_bumper) {
                intakeTransfer.setPower(-0.8);
            } else {
                intakeTransfer.setPower(0);
            }


            // Loading Controls

            if (gamepad2.dpad_up) {
                loadingTransfer.setPower(0.8); // Adjust position as needed
            } else if (gamepad2.dpad_down) {
                loadingTransfer.setPower(-0.5); // Adjust position as needed
            } else {
                loadingTransfer.setPower(0); // Adjust position as needed
            }


        }
    }
}
