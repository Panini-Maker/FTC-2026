package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingToleranceTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgunTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.AprilTag;
import org.firstinspires.ftc.teamcode.lib.CameraMovement;
import org.firstinspires.ftc.teamcode.lib.ShooterController;
import org.firstinspires.ftc.teamcode.lib.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


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

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Turret turretController = new Turret(turret, telemetry);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        Servo light = hardwareMap.get(Servo.class, "light");

        hoodServo.setDirection(Servo.Direction.REVERSE);

        // Configure odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM); // Set offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        int latchState = 0;

        double power = 0.8; //Used teo limit speed for testing/safety

        double turretPower = 1.0;
        boolean shooterMode = true;
        boolean targetIsRed = true;
        double shooterPower;
        int target = redTagID;
        boolean isAligned = false;
        double currentHeading = 0.0;
        double distanceToGoal = 0.0;

        //Auto Aim and Auto Shoot variables
        boolean autoAim = false;
        boolean toggleAutoAim = true;
        boolean autoShoot = false;
        boolean toggleAutoShoot = true;
        boolean safeShooting = true;

        //May be useful later
        double currentXOdo;
        double currentYOdo;
        double currentHeadingOdo;

        // Recalibrate Odometry
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Adjust for lighting conditions
        //CameraMovement camera = new CameraMovement(frontLeft, frontRight, backRight, backLeft, leftShooter, rightShooter, hoodServo, leftLatch, rightLatch, turret , odo, intake, voltageSensor, telemetry, tagProcessor);


        waitForStart();
        resetRuntime();

        ElapsedTime autoAimCoolDown = new ElapsedTime();
        while (opModeIsActive()) {
            // Get position

            odo.update();

            if (gamepad2.yWasPressed()) {
                shooterMode = !shooterMode;
            }

            if (gamepad2.aWasPressed()) {
                targetIsRed = !targetIsRed;
            }

            if (targetIsRed) {
                target = redTagID;
                telemetry.addData("Target Color:", "Red");
            } else {
                target = blueTagID;
                telemetry.addData("Target Color:", "Blue");
            }

            if (shooterMode) {
                shooterPower = shotgunTeleOp;
                telemetry.addData("Shooter Mode:", "Shotgun");
                //shooter.setPIDConstants(0.0025,0.00099,0);
                hoodServo.setPosition(0.4);
            } else {
                shooterPower = sniper;
                telemetry.addData("Shooter Mode", "Sniper");
                //shooter.setPIDConstants(0.0025,0.00099,0);
                hoodServo.setPosition(0.6);
            }

            //Consider using toggle with right bumper/trigger instead of holding right trigger
            /*
            if (gamepad1.right_bumper && toggleAutoAim) {
                autoAim = !autoAim;
                //Turn off autoShoot when autoAim is off
                if (!autoAim) {
                    autoShoot = false;
                }
                toggleAutoAim = false;
            } else {
                toggleAutoAim = true;
            }

             */

            autoAim = gamepad2.dpad_right;

            if (!autoAim) {
                autoShoot = false;
            }

            /*
            if (gamepad1.left_bumper && toggleAutoShoot) {
                autoShoot = !autoShoot;
                //Turn autoAim on when autoShoot is on
                if (autoShoot) {
                    autoAim = true;
                }

                toggleAutoShoot = false;
            } else {
                toggleAutoShoot = true;
            }

             */

            autoShoot = gamepad2.dpad_left;

            if (autoShoot) {
                autoAim = true;
            }

            if (gamepad1.yWasPressed()) {
                safeShooting = !safeShooting;
            }

            if (autoAim && (autoAimCoolDown.milliseconds() > 100) && !isAligned) {
                sleep (100);
                if ((tagProcessor.getFreshDetections() != null) && !tagProcessor.getDetections().isEmpty()) {
                    sleep(100);
                    for (AprilTagDetection tag : tagProcessor.getDetections()) {
                        if (tag.id == target) {
                            // Skip adjustment if bearing is within 1 degree tolerance
                            if (Math.abs(tag.ftcPose.bearing) <= 1.0) {
                                telemetry.addData("Bearing to Target", tag.ftcPose.bearing);
                                telemetry.addData("Auto Aim Status", "Within tolerance, skipping");
                                isAligned = true;
                                distanceToGoal = tag.ftcPose.range + 18; //Estimate distance to goal from tag range
                                break;
                            }
                            isAligned = false;
                            double adjustment = currentHeading + tag.ftcPose.bearing;
                            telemetry.addData("Bearing to Target", tag.ftcPose.bearing);
                            if (adjustment > turretLimitCCW) {
                                adjustment = turretLimitCCW;
                            } else if (adjustment < turretLimitCW) {
                                adjustment = turretLimitCW;
                            }
                            currentHeading = turretController.spinToHeading(adjustment, turretPower);
                            break;
                        }
                    }
                }
            }

            // Reset alignment status when robot moves or auto-aim is toggled off
            if (!autoAim) {
                isAligned = false;
            }

            if (gamepad2.right_bumper) {
                if (!(leftLatch.getPosition() == 1) && (leftShooter.getPower() == 0) && (rightShooter.getPower() == 0)) {
                    latchState = 0;
                }
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-0.8);
            } else {
                intake.setPower(0);
            }

            if ((gamepad2.right_trigger > 0) || (isAligned && autoShoot)) {
                if (!(leftLatch.getPosition() == 0)) {
                    latchState = 1;
                }
                shooter.runShooter(shooterPower);
            } else if (gamepad2.left_trigger > 0) {
                shooter.resetPID();
                leftShooter.setPower(-0.3);
                rightShooter.setPower(-0.3);
                //for intaking from human players
            } else {
                shooter.resetPID();
                shooter.stopShooter();
            }

            if (gamepad1.left_trigger > 0) {
                power = 0.6;
            } else if (gamepad1.right_trigger > 0) {
                power = 1;
            } else {
                power = 0.8;
            }

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

            if ((y > 0.1) || (y < -0.1) || (x > 0.1) || (x < -0.1) || (rx > 0.1) || (rx < -0.1)) {
                //Resets auto-aim cooldown if driver is moving the robot
                //Auto aim relies on robot being stationary for accuracy
                autoAimCoolDown.reset();
                isAligned = false; // Reset alignment when robot moves
            }

            // Intake and Transfer Controls
            // right trigger to shooting balls
            //left trigger for intaking and transferring balls
            // if both triggers are pressed, the robot will do both actions simultaneously
            // right button is the outtake in case we intake too many artifacts

            if (gamepad2.x && (currentHeading <= turretLimitCCW)) {
                currentHeading = turretController.spinToHeading(currentHeading + 10, turretPower);
            } else if (gamepad2.b && (currentHeading >= turretLimitCW)) {
                currentHeading = turretController.spinToHeading(currentHeading - 10, turretPower);
            } else {
                turretController.stopTurret();
            }

            if (gamepad2.dpad_down) {
                latchState = 0;
            } else if (gamepad2.dpad_up) {
                latchState = 1;
            }

            //light for shooter status
            double avgShooterVel = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;

            if (avgShooterVel > (shooterPower - shootingToleranceTeleOp) && avgShooterVel < (shooterPower + shootingToleranceTeleOp)) {
                light.setPosition(0.5);
                if (((gamepad2.right_bumper && (gamepad2.right_trigger > 0.1)) && safeShooting) || autoShoot) {
                    intake.setPower(1);
                    latchState = 1;
                }
            } else {
                light.setPosition(0.28);
                if (((gamepad2.right_bumper && (gamepad2.right_trigger > 0.1)) && safeShooting) || autoShoot) {
                    intake.setPower(0);
                }
            }

            rightLatch.setPosition(latchState);
            leftLatch.setPosition(1 - latchState);

            //Pose2D pos = odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);
            telemetry.addData("Auto Aim", autoAim);
            telemetry.addData("Auto Shoot", autoShoot);
            telemetry.addData("Safe Shooting", safeShooting);
            telemetry.addData("Turret Target Heading", currentHeading);
            telemetry.addData("Turret Heading", currentHeading);
            telemetry.addData("Turret Power", turret.getPower());
            telemetry.addData("Left Shooter Velocity", leftShooter.getVelocity());
            telemetry.addData("Left Shooter Power", leftShooter.getPower());
            telemetry.addData("Right Shooter Velocity", rightShooter.getVelocity());
            telemetry.addData("Right Shooter Power", rightShooter.getPower());
            telemetry.update();
        }
    }
}
