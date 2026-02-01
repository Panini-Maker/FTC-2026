package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class DrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            //Start Debug Section
            //Set to true to enable debug controls
            boolean debug = true;
            if (debug) {
                if (gamepad1.dpad_up) {
                    frontLeft.setPower(1);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                } else if (gamepad1.dpad_down) {
                    backLeft.setPower(1);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                } else if (gamepad1.y) {
                    frontRight.setPower(1);
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                } else if (gamepad1.a) {
                    backRight.setPower(1);
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                }
            }

            //End Debug Section

            double multiplier = 0.5; //Used to limit speed for testing/safety
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

            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.update();

        }
    }
}
