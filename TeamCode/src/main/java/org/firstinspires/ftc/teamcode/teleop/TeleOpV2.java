package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndTurretHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndX;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndY;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterIdle;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingToleranceTeleOp;
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

        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Turret turretController = new Turret(turret, telemetry);

        // Initialize AutoAim (default to red team, can be toggled)
        AutoAim autoAimController = new AutoAim(turret, telemetry, targetIsRed);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        Servo light = hardwareMap.get(Servo.class, "light");

        hoodServo.setDirection(Servo.Direction.REVERSE);

        // Configure odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM); // Set offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        Pose2D pos;

        RobotActions robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake,
                rightLatch, leftLatch, hoodServo, light);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Presets
        double drivetrainPower = 0.9;

        // Mode Variables
        int latchState = 0;
        double hoodState;
        double intakePower;
        double lightState;
        double turretPower = 1.0;

//        boolean targetIsRed = true;
        double shooterSpeed = 0;
        int target = redTagID;
        boolean isAligned = false;
        double currentHeading = 0.0;
        double distanceToGoal;
        double calculatedTargetAngle;
        double currentTurretAngle;

        //Auto Aim and Auto Shoot variables
        boolean autoAim = false;
        boolean autoShoot = false;
        boolean safeShooting = false;

        // Odometry
        double currentXOdo;
        double currentYOdo;
        double currentHeadingOdo;

        // Initialize odometry
        // Only reset if we don't have saved position from autonomous
        if (autoEndX == 0 && autoEndY == 0 && autoEndHeading == 0) {
            // Wait for IMU to calibrate
            odo.resetPosAndIMU();
            sleep(250);
            telemetry.addData("Odometry", "Reset to origin, IMU calibrating...");
        } else {
            // Have saved position from autonomous - set it without full reset
            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH, autoEndX, autoEndY, AngleUnit.DEGREES, autoEndHeading));
            telemetry.addData("Odometry", "Loaded from Auto: X=%.1f, Y=%.1f, H=%.1f", autoEndX, autoEndY, autoEndHeading);
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
            if (gamepad1.optionsWasPressed()){
                odo.resetPosAndIMU();
            }
            // Get position
            odo.update();
            pos = odo.getPosition();

            currentXOdo = pos.getX(DistanceUnit.INCH);
            currentYOdo = pos.getY(DistanceUnit.INCH);
            currentHeadingOdo = pos.getHeading(AngleUnit.DEGREES);

            // Goal Color
            if (gamepad1.backWasPressed()) {
                targetIsRed = !targetIsRed;
                autoAimController.setTeam(targetIsRed); // Update auto aim target
            }
            if (targetIsRed) {
                target = redTagID;
                telemetry.addData("Target Color:", "Red");
            } else {
                target = blueTagID;
                telemetry.addData("Target Color:", "Blue");
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
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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
                shooterSpeed = shooterIdle;
            }

            // Turret
            calculatedTargetAngle = autoAimController.calculateTargetAngle(currentXOdo, currentYOdo, currentHeadingOdo);
            currentTurretAngle = turretController.getCurrentHeading();

            // Light for shooter status
            double avgShooterVel = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
            if (Math.abs(avgShooterVel - shooterSpeed) < shootingToleranceTeleOp) {
                lightState = 0.5;
            } else {
                lightState = 0.28;
            }

            /// SET POWER =================================================================================================================

//            robot.driveRobotCentric(y, x, rx, drivetrainPower);
            robot.driveFieldCentric(pos, y, x, rx, drivetrainPower);

            robot.setLatch(latchState);
            hoodServo.setPosition(hoodState);
            shooter.runShooter(shooterSpeed);
            intake.setPower(intakePower);
            turretController.spinToHeadingLoop(calculatedTargetAngle, turretPower);
            light.setPosition(lightState);

            /// TELEMETRY ==================================================================================================================

            telemetry.addData("Robot Speed", drivetrainPower);
            telemetry.addData("Robot X", currentXOdo);
            telemetry.addData("Robot Y", currentYOdo);
            telemetry.addData("Robot Heading", currentHeadingOdo);
            telemetry.addData("Distance from Goal", distanceToGoal);
            telemetry.addData("Auto Aim", autoAim);
            telemetry.addData("Auto Aim Aligned", isAligned);
            telemetry.addData("Auto Shoot", autoShoot);
            telemetry.addData("Safe Shooting", safeShooting);
            telemetry.addData("Turret Target Heading", currentHeading);
            telemetry.addData("Turret Actual Heading", autoAimController.getCurrentTurretHeading());
            telemetry.addData("Turret Power", turret.getPower());
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
