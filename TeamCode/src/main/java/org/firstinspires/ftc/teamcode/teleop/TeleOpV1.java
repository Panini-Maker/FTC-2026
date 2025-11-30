package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.CameraMovement;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.lib.AprilTag;

import java.util.Locale;


@TeleOp
public class TeleOpV1 extends LinearOpMode {

    GoBildaPinpointDriver odo;
    CameraMovement camera;

    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotor transfer = hardwareMap.dcMotor.get("loading");
        DcMotor intake = hardwareMap.dcMotor.get("intakeTransfer");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM); // Set offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        boolean mode = true;
        boolean targetIsRed = true;
        double shooterPower;
        int target = redTagID;
        boolean isAligned = false;

        // Recalibrate Odometry
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Adjust for lighting conditions
        CameraMovement camera = new CameraMovement(frontLeft, frontRight, backRight, backLeft, leftShooter, odo, intake, transfer, voltageSensor, telemetry, tagProcessor);



        waitForStart();
        resetRuntime();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {

            // Get postition

            odo.update();

            if (gamepad2.yWasPressed()) {
                mode = !mode;

            }

            if (gamepad2.aWasPressed()) {
                targetIsRed = !targetIsRed;
            }

            if(targetIsRed) {
                target = redTagID;
                telemetry.addData("Target Color:", "Red");
            } else {
                target = blueTagID;
                telemetry.addData("Target Color:", "Blue");
            }

            if (mode) {
                shooterPower = shotgun;
                telemetry.addData("Shooter Mode:", "Shotgun");
            } else {
                shooterPower = sniper;
                telemetry.addData("Shooter Mode", "Sniper");
            }
            //boolean wasRightTriggerPressed = false;

            if (gamepad2.right_trigger > 0) {
                leftShooter.setVelocity(shooterPower);

                if (!isAligned) {
                    // Trigger just pressed - start shooting once
                    isAligned = camera.turnToAprilTagNoOdo(target);
                }

                // Keep aligning while trigger is held


            } else if (gamepad2.left_trigger > 0) {
                leftShooter.setPower(-0.3); //for intaking from human players
                //wasRightTriggerPressed = false;

            } else {
                leftShooter.setPower(0);
                isAligned = false;
                //wasRightTriggerPressed = false;
            }

            timer.reset();

            double power = 0.8; //Used teo limit speed for testing/safety
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Turning

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double frontLeftPower = (y + x + rx) / Math.max(Math.abs(y + x + rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
            double frontRightPower = (y - x - rx) / Math.max(Math.abs(y - x - rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
            double backLeftPower = (y - x + rx) / Math.max(Math.abs(y - x + rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
            double backRightPower = (y + x - rx) / Math.max(Math.abs(y + x - rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());

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
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }


            // Loading Controls

            if (gamepad2.dpad_up) {
                transfer.setPower(0.8); // Adjust position as needed
                intake.setPower(0.8);
            } else if (gamepad2.dpad_down) {
                transfer.setPower(-0.5); // Adjust position as needed
            } else {
                transfer.setPower(0); // Adjust position as needed
            }


            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();


        }
    }
}
