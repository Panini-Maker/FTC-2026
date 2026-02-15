package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndTurretHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndX;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndY;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoResetPosRed;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterIdle;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingToleranceTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgunTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.targetIsRed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.RobotActions;
import org.firstinspires.ftc.teamcode.lib.ShooterController;
import org.firstinspires.ftc.teamcode.lib.Turret;


@TeleOp
public class TeleOpV2 extends LinearOpMode {

    GoBildaPinpointDriver odo;

    @Override

    public void runOpMode() throws InterruptedException {
        // Motors and Servos
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        Servo light = hardwareMap.get(Servo.class, "light");

        hoodServo.setDirection(Servo.Direction.REVERSE);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM); // Set offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Activate External Classes
        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);
        Turret turretController = new Turret(turret, telemetry);
        AutoAim autoAimController = new AutoAim(turret, telemetry, targetIsRed);
        RobotActions robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake,
                rightLatch, leftLatch, hoodServo, light);

        // Presets
        double drivetrainPower = 0.9;
        double turretPower = 1.0;
        double manualTurretHeading = 0.0;

        // Mode Variables
        Pose2D pos;
        int latchState = 0;
        double hoodState;
        double intakePower;
        double lightState;
        double calculatedTargetAngle;
        double shooterSpeed;

        // Info Variables
        int target = redTagID;
        boolean isAligned = false;
        double currentHeading = 0.0;
        double distanceToGoal;

        boolean manualMode = false;
        boolean shootingWhileMoving = true;

        // Odometry
        double currentXOdo;
        double currentYOdo;
        double currentHeadingOdo;
        double x_velocity;
        double y_velocity;
        double heading_velocity;

        // Initialize odometry
        // Only reset if we don't have saved position from autonomous
        if (autoEndX == 0 && autoEndY == 0 && autoEndHeading == 0) {
            // Wait for IMU to calibrate
            odo.resetPosAndIMU();
            sleep(250);
            telemetry.addData("Odometry", "Reset to origin, IMU calibrating...");
        } else {
            // Have saved position from autonomous - set it without full reset
            odo.setPosition(new Pose2D(DistanceUnit.INCH, autoEndX, autoEndY, AngleUnit.DEGREES, autoEndHeading));
            Pose2D startPose = odo.getPosition();
            telemetry.addData("Odometry", "Loaded from Auto: X=%.1f, Y=%.1f, H=%.1f", startPose.getX(DistanceUnit.INCH), startPose.getY(DistanceUnit.INCH), startPose.getHeading(AngleUnit.RADIANS));
        }

        // Find correct april tag
        if (targetIsRed) {
            target = redTagID;
            telemetry.addData("Target Color:", "Red");
        } else {
            target = blueTagID;
            telemetry.addData("Target Color:", "Blue");
        }

        // Initialize turret heading from autonomous (if available)
        currentHeading = autoEndTurretHeading;

        // Get position
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            /// LOCALIZATION ==================================================================================================================

            // Reset odo midmatch if needed
            if (gamepad1.startWasPressed() && gamepad1.xWasPressed()) {
                if (targetIsRed) {
                    odo.setPosition(new Pose2D(DistanceUnit.INCH, odoResetPosRed.x, odoResetPosRed.y, AngleUnit.DEGREES, 90.0));
                } else {
                    odo.setPosition(new Pose2D(DistanceUnit.INCH, -odoResetPosRed.x, odoResetPosRed.y, AngleUnit.DEGREES, 90.0));
                }
            }
            // Get position
            odo.update();
            pos = odo.getPosition();
            x_velocity = odo.getVelX(DistanceUnit.INCH);
            y_velocity = odo.getVelY(DistanceUnit.INCH);
            heading_velocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            // Adjust robot position for shooting while moving
            if (shootingWhileMoving) {
                pos = robot.shootWhileMoving(pos, x_velocity, y_velocity, heading_velocity);
            }

            currentXOdo = pos.getX(DistanceUnit.INCH);
            currentYOdo = pos.getY(DistanceUnit.INCH);
            currentHeadingOdo = pos.getHeading(AngleUnit.DEGREES);

            // Change Goal Color Midmatch
            if (gamepad1.startWasPressed() && gamepad1.yWasPressed()) {
                targetIsRed = !targetIsRed;
                autoAimController.setTeam(targetIsRed); // Update auto aim target

                // reset april tag
                if (targetIsRed) {
                    target = redTagID;
                    telemetry.addData("Target Color:", "Red");
                } else {
                    target = blueTagID;
                    telemetry.addData("Target Color:", "Blue");
                }
            }

            /// CALCULATE =================================================================================================================

            autoAimController.updateRobotPosition(currentXOdo, currentYOdo, currentHeading, autoAimController.getCurrentTurretHeading());
            distanceToGoal = robot.getDistanceFromGoal(pos, targetIsRed);
            hoodState = robot.getShooterAngle(distanceToGoal);


            /// CONTROLS ==================================================================================================================

            // right trigger run shooter, turn turret, open latch
            // right bumper run intake, close latch
            // left bumper run outtake
            // a switch speed
            // joystick move bot
            // options --> manual mode

            // Speed toggle
            if (gamepad1.aWasPressed()) {
                drivetrainPower = 1.5 - drivetrainPower;
            }
            // Drivetrain inputs
            double x;
            double y;
            if (!targetIsRed && !manualMode) {
                y = gamepad1.left_stick_y; // Forward/Backward
                x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            } else {
                y = -gamepad1.left_stick_y; // Forward/Backward
                x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing and reverse for blue side
            }
            double rx = gamepad1.right_stick_x; // Turning

            // Intake
            if (gamepad1.right_bumper) {
                intakePower = 1;
                latchState = 0; // Close latch
            } else if (gamepad1.left_bumper) {
                intakePower = -0.5; // Less to prevent artifacts from going far away
            } else {
                intakePower = 0;
            }

            // Shooter controls
            // If right trigger, run shooter + open latch, else run slow speed (for faster accel)
            if (gamepad1.right_trigger > 0) {
                shooterSpeed = robot.getShooterRPM(distanceToGoal);
                latchState = 1;
            } else {
                shooter.stopVelocityPID();
                shooter.stopShooter();
                shooterSpeed = 0;
            }

            // Turret
            calculatedTargetAngle = autoAimController.calculateTargetAngle(currentXOdo, currentYOdo, currentHeadingOdo);

            // Light for shooter status
            double avgShooterVel = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
            if (Math.abs(avgShooterVel - shooterSpeed) < shootingToleranceTeleOp) {
                lightState = 0.5;
            } else {
                lightState = 0.28;
            }

            // Manual turret control overrides auto aim
            if (gamepad1.backWasPressed() && gamepad1.startWasPressed()) {
                manualMode = !manualMode;
            }

            if (manualMode) {
                calculatedTargetAngle = manualTurretHeading;
                shooterSpeed = shotgunTeleOp;
            }

            if (gamepad1.yWasPressed()) {
                if (shooterSpeed == shotgunTeleOp) {
                    shooterSpeed = sniper;
                    hoodState = 0.5;
                } else if (shooterSpeed == sniper) {
                    shooterSpeed = shotgunTeleOp;
                    hoodState = 0.42;
                }
            }

            /// SET POWER =================================================================================================================

//            robot.driveRobotCentric(y, x, rx, drivetrainPower);
            if (!manualMode) {
                robot.driveFieldCentric(pos, y, x, rx, drivetrainPower);
            } else {
                robot.driveRobotCentric(y, x, rx, drivetrainPower);
            }
            robot.setLatch(latchState);
            hoodServo.setPosition(hoodState);
            if (shooterSpeed != 0) {
                //shooter.runShooter(shooterSpeed);
                shooter.setVelocityPID(shooterSpeed);
            }
            intake.setPower(intakePower);
            turretController.spinToHeadingLoop(calculatedTargetAngle, turretPower);
            light.setPosition(lightState);

            /// TELEMETRY ==================================================================================================================

            telemetry.addData("Team Red?", targetIsRed);
            telemetry.addData("Shooting While Moving", shootingWhileMoving);

            telemetry.addData("Robot Speed", drivetrainPower);
            telemetry.addData("Robot X", currentXOdo);
            telemetry.addData("Robot Y", currentYOdo);
            telemetry.addData("Robot Heading", currentHeadingOdo);
            telemetry.addData("Distance from Goal", distanceToGoal);
            telemetry.addData("Auto Aim Aligned", isAligned);
            telemetry.addData("Turret Target Heading", currentHeading);
            telemetry.addData("Turret Actual Heading", autoAimController.getCurrentTurretHeading());
            telemetry.addData("Shooter Target RPM", shooterSpeed);
            telemetry.addData("Shooter Actual RPM", avgShooterVel);
            telemetry.update();
        }

        // Cleanup - stop all threads when OpMode ends
        autoAimController.stopAutoAim();
        shooter.stopVelocityPID();
        turretController.stopVelocityPID();
        saveEndPosition(0, 0, 0, 0);
    }
}
