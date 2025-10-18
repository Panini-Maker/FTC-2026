package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="2-Player Outreach Code")
public class TwoPlayerOutreachCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        CRServo intake = hardwareMap.crservo.get("Intake");
        Servo rotation = hardwareMap.servo.get("Rotation");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        rotation.setPosition(0);

        while (opModeIsActive()) {

            double speed_multiplier = 0.3; //Used to limit speed for testing/safety
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Turning

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator  * speed_multiplier ;
            double backLeftPower = (y - x + rx) / denominator * speed_multiplier;
            double frontRightPower = (y - x - rx) / denominator * speed_multiplier;
            double backRightPower = (y + x - rx) / denominator * speed_multiplier;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            if (gamepad2.right_trigger > 0.5) {
                intake.setPower(-1);
            } else if (gamepad2.left_trigger > 0.5) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
                }

            if (gamepad2.dpad_down) {
                rotation.setPosition(0.9);
            } else if (gamepad2.dpad_up) {
                rotation.setPosition(0.20);
            }

        }
    }
}
