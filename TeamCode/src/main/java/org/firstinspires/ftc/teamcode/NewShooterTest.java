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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.ShooterController;

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

        // Shooter
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turret
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo hood = hardwareMap.get(Servo.class, "hoodServo");

        // Drivetrain
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Light
        Servo light = hardwareMap.get(Servo.class, "light");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Presets
        double COUNTS_PER_MOTOR_REV = 28.0;


        // variables

        long shooterSpeed;

        double intakeSpeed = 0.8;

        int latch = 0; // 0 = closed, 1 = open

        double drivetrainMaxPower = 0.8;

        double turretPower;

        double hoodAngle = 1; // 1 is all the way down, 0 is all the way up

        rightLatch.setPosition(1 - latch); // 1 is closed, 0 is open
        leftLatch.setPosition(latch); // 0 is closed, 1 is open
        hood.setPosition(hoodAngle);

        ElapsedTime cooldown = new ElapsedTime();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /// Control Settings

            // Shooter
            if (gamepad2.right_trigger > 0) {
                /// IMPORTANT: leftShooter must be set at negative, to reverse direction
                shooterSpeed = 2000;
            } else if (gamepad2.left_trigger > 0) {
                shooterSpeed = -300;
            } else {
                shooterSpeed = 0;
            }

            // Latch
            if (gamepad2.right_bumper && (cooldown.milliseconds() > 300)) {
                latch = 1 - latch;
                cooldown.reset();
            }

            // Hood (for testing only

            if (gamepad2.dpad_up) {
                if (hoodAngle > 0) {
                    hoodAngle -= 0.05;
                }
            } else if (gamepad2.dpad_down) {
                if (hoodAngle < 1) {
                    hoodAngle += 0.05;
                }
            }

            if (gamepad1.right_trigger > 0) {
                intakeMotor.setPower(intakeSpeed);
            } else if (gamepad1.left_trigger > 0) {
                intakeMotor.setPower(-intakeSpeed);
            } else {
                intakeMotor.setPower(0);
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * drivetrainMaxPower;
            double backLeftPower = (y - x + rx) / denominator * drivetrainMaxPower;
            double frontRightPower = (y - x - rx) / denominator * drivetrainMaxPower;
            double backRightPower = (y + x - rx) / denominator * drivetrainMaxPower;

            turretPower = gamepad2.right_stick_x * 0.5; // 0.5 for testing speed

            ///  Set Power to Motors/Servos

            rightShooter.setVelocity(shooterSpeed);
            leftShooter.setVelocity(shooterSpeed);

            turretMotor.setPower(turretPower);

            rightLatch.setPosition(1 - latch);
            leftLatch.setPosition(latch);

            hood.setPosition(hoodAngle);

            double avgShooterVel = (leftShooter.getVelocity() + rightShooter.getVelocity())/2;

            if (avgShooterVel > 1950 && avgShooterVel < 2050) {
                light.setPosition(0.5);
            } else {
                light.setPosition(0.28);
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Shooter Velocity", avgShooterVel);
            telemetry.addData("Shooter Angle", hoodAngle);
            telemetry.addData("Latch", latch);
            telemetry.update();

        }

    }
}
