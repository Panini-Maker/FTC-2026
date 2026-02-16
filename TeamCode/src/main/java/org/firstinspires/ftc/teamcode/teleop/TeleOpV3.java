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
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingToleranceTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgunTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.targetIsRed;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.AprilTag;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Camera;
import org.firstinspires.ftc.teamcode.lib.RobotActions;
import org.firstinspires.ftc.teamcode.lib.ShooterController;
import org.firstinspires.ftc.teamcode.lib.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TeleOpV3 - Enhanced TeleOp with multiple operating modes
 *
 * ALLIANCE SELECTION (during init):
 * - DPad Up/Down to scroll between RED and BLUE alliance
 * - Selection is locked once OpMode starts
 *
 * OPERATING MODES:
 * - STANDARD: Full auto-aim, field-centric driving, 1 driver
 * - RELOCALIZATION: Camera searching for AprilTag to reset pose
 * - MANUAL: Robot-centric driving, 2 drivers, turret presets
 *
 * MODE SWITCHING (during OpMode):
 * - DPad Up: Go up ranking (MANUAL -> RELOCALIZATION -> STANDARD)
 * - DPad Down: Go down ranking (STANDARD -> RELOCALIZATION -> MANUAL)
 *
 * LIGHT COLORS:
 * - Blue (0.611): STANDARD mode
 * - Yellow (0.388): RELOCALIZATION mode
 * - Red (0.28): MANUAL mode
 * - Green (0.5): Shooter ready (overrides mode color when shooting)
 */
@TeleOp(name = "TeleOp V3", group = "Competition")
public class TeleOpV3 extends LinearOpMode {

    // Operating modes
    private enum OperatingMode {
        MANUAL,         // Rank 0: Robot-centric, 2 drivers, presets
        RELOCALIZATION, // Rank 1: Searching for AprilTag
        STANDARD        // Rank 2: Full auto-aim, field-centric
    }

    // Light colors for each mode
    private static final double LIGHT_BLUE = 0.611;    // Standard mode
    private static final double LIGHT_YELLOW = 0.388;  // Relocalization mode
    private static final double LIGHT_RED = 0.28;      // Manual mode
    private static final double LIGHT_GREEN = 0.5;     // Shooter ready

    // Relocalization thresholds
    private static final double RELOCALIZATION_SPEED_THRESHOLD = 0.1; // inches/second for linear velocity
    private static final double RELOCALIZATION_ROTATION_THRESHOLD = 0.1; // degrees/second for rotational velocity
    private static final int RED_GOAL_TAG_ID = 24;  // AprilTag ID for red goal
    private static final int BLUE_GOAL_TAG_ID = 20; // AprilTag ID for blue goal

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
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        // Note: ShooterController sets direction and run mode

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
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Activate External Classes
        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, shooterKf, voltageSensor, telemetry);
        Turret turretController = new Turret(turret, telemetry);
        RobotActions robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake,
                rightLatch, leftLatch, hoodServo, light);

        // Initialize AprilTag processor for relocalization
        AprilTagProcessor aprilTagProcessor = null;
        boolean aprilTagAvailable = false;
        try {
            aprilTagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
            aprilTagAvailable = (aprilTagProcessor != null);
        } catch (Exception e) {
            telemetry.addLine("WARNING: AprilTag initialization failed!");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            aprilTagAvailable = false;
        }

        // Initialize Camera for pose calculation (set camera offsets as needed)
        Camera cameraRelocalization = new Camera(5.6349839, 0.78702);

        // Presets
        double drivetrainPower = 0.9;
        double turretPower = 1.0;
        double manualTurretHeading = 0.0;

        // Mode Variables
        Pose2D pos;
        int latchState = 0;
        double hoodState;
        double intakePower;
        double lightState = 0;
        double calculatedTargetAngle = 0;
        double shooterSpeed = 0;

        // Info Variables
        int target = redTagID;
        double currentHeading = 0.0;
        double distanceToGoal;

        // Operating mode
        OperatingMode currentMode = OperatingMode.STANDARD;
        boolean shootingWhileMoving = true;

        // Alliance selection (during init)
        boolean allianceIsRed = targetIsRed;
        ElapsedTime debounceTimer = new ElapsedTime();

        // Odometry
        double currentXOdo;
        double currentYOdo;
        double currentHeadingOdo;
        double x_velocity;
        double y_velocity;
        double heading_velocity;

        // Manual mode turret preset tracking
        boolean usingSniperPreset = false;

        // ==================== INIT PHASE: ALLIANCE SELECTION ====================
        telemetry.addLine("╔══════════════════════════════════╗");
        telemetry.addLine("║      TeleOp V3 - Alliance Select      ║");
        telemetry.addLine("╠══════════════════════════════════╣");
        telemetry.addLine("║  Use DPad UP/DOWN to select alliance  ║");
        telemetry.addLine("╚══════════════════════════════════╝");
        telemetry.update();

        // Alliance selection loop during init
        while (!isStarted() && !isStopRequested()) {
            // Handle alliance selection with debounce
            if (debounceTimer.milliseconds() > 250) {
                if (gamepad1.dpad_up) {
                    allianceIsRed = true;
                    debounceTimer.reset();
                } else if (gamepad1.dpad_down) {
                    allianceIsRed = false;
                    debounceTimer.reset();
                }
            }

            // Display alliance selection
            telemetry.addLine("╔══════════════════════════════════╗");
            telemetry.addLine("║      TeleOp V3 - Alliance Select      ║");
            telemetry.addLine("╠══════════════════════════════════╣");

            if (allianceIsRed) {
                telemetry.addLine("║            ▶ RED ALLIANCE ◀            ║");
                telemetry.addLine("║              Blue Alliance              ║");
            } else {
                telemetry.addLine("║              Red Alliance               ║");
                telemetry.addLine("║           ▶ BLUE ALLIANCE ◀           ║");
            }

            telemetry.addLine("╠══════════════════════════════════╣");
            telemetry.addLine("║  DPad UP = Red, DPad DOWN = Blue  ║");
            telemetry.addLine("╚══════════════════════════════════╝");
            telemetry.update();

            sleep(50);
        }

        // Lock in alliance selection
        targetIsRed = allianceIsRed;

        // Initialize AutoAim with selected alliance
        AutoAim autoAimController = new AutoAim(turret, telemetry, targetIsRed);

        // Initialize odometry
        if (autoEndX == 0 && autoEndY == 0 && autoEndHeading == 0) {
            odo.resetPosAndIMU();
            sleep(250);
        } else {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, autoEndX, autoEndY, AngleUnit.DEGREES, autoEndHeading));
        }

        // Set target based on alliance
        if (targetIsRed) {
            target = redTagID;
        } else {
            target = blueTagID;
        }

        // Initialize turret heading from autonomous
        currentHeading = autoEndTurretHeading;

        // ==================== MAIN OPMODE LOOP ====================
        resetRuntime();
        debounceTimer.reset();

        while (opModeIsActive()) {

            // ==================== MODE SWITCHING ====================
            // DPad Up: Go up ranking (MANUAL -> RELOCALIZATION -> STANDARD)
            // DPad Down: Go down ranking (STANDARD -> RELOCALIZATION -> MANUAL)
            if (debounceTimer.milliseconds() > 300) {
                if (gamepad1.dpad_up) {
                    switch (currentMode) {
                        case MANUAL:
                            currentMode = OperatingMode.RELOCALIZATION;
                            break;
                        case RELOCALIZATION:
                            currentMode = OperatingMode.STANDARD;
                            break;
                        case STANDARD:
                            // Already at top, stay here
                            break;
                    }
                    debounceTimer.reset();
                } else if (gamepad1.dpad_down) {
                    switch (currentMode) {
                        case STANDARD:
                            currentMode = OperatingMode.RELOCALIZATION;
                            break;
                        case RELOCALIZATION:
                            currentMode = OperatingMode.MANUAL;
                            break;
                        case MANUAL:
                            // Already at bottom, stay here
                            break;
                    }
                    debounceTimer.reset();
                }
            }

            // ==================== LOCALIZATION ====================
            // Reset odo midmatch if needed
            if (gamepad1.start && gamepad1.x) {
                if (targetIsRed) {
                    odo.setPosition(new Pose2D(DistanceUnit.INCH, odoResetPosRed.x, odoResetPosRed.y, AngleUnit.DEGREES, 90.0));
                } else {
                    odo.setPosition(new Pose2D(DistanceUnit.INCH, -odoResetPosRed.x, odoResetPosRed.y, AngleUnit.DEGREES, 90.0));
                }
            }

            // Get position
            odo.update();
            Pose2D currentPose = odo.getPosition();
            x_velocity = odo.getVelX(DistanceUnit.INCH);
            y_velocity = odo.getVelY(DistanceUnit.INCH);
            heading_velocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            // Get shoot-while-moving offsets and create adjusted pose (only in STANDARD mode)
            Pose2D adjustedPose;
            double[] swmOffsets = {0, 0, 0};
            if (currentMode == OperatingMode.STANDARD && shootingWhileMoving) {
                swmOffsets = robot.getShootWhileMovingOffsets(x_velocity, y_velocity, heading_velocity);
                adjustedPose = robot.applyShootWhileMovingOffsets(currentPose, swmOffsets);
            } else {
                adjustedPose = currentPose;
            }

            // Current pose values (from odometry, unmodified)
            currentXOdo = currentPose.getX(DistanceUnit.INCH);
            currentYOdo = currentPose.getY(DistanceUnit.INCH);
            currentHeadingOdo = currentPose.getHeading(AngleUnit.DEGREES);

            // Adjusted pose values (for auto-aim calculations)
            double adjustedX = adjustedPose.getX(DistanceUnit.INCH);
            double adjustedY = adjustedPose.getY(DistanceUnit.INCH);
            double adjustedHeading = adjustedPose.getHeading(AngleUnit.DEGREES);

            // Use currentPose for driving, adjustedPose for auto-aim
            pos = currentPose;

            // ==================== RELOCALIZATION MODE ====================
            if (currentMode == OperatingMode.RELOCALIZATION) {
                // Check if AprilTag is available
                if (!aprilTagAvailable || aprilTagProcessor == null) {
                    telemetry.addLine(">>> RELOCALIZATION UNAVAILABLE <<<");
                    telemetry.addLine("AprilTag processor not initialized.");
                    telemetry.addLine("Press DPad Down to switch to MANUAL mode.");
                } else {
                    // Calculate total speed (linear and rotational)
                    double linearSpeed = Math.sqrt(x_velocity * x_velocity + y_velocity * y_velocity);
                    double rotationalSpeed = Math.abs(heading_velocity);

                    // Get AprilTag detections regardless of speed (for telemetry)
                    List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

                    // Only attempt relocalization when robot is nearly stationary
                    if (linearSpeed < RELOCALIZATION_SPEED_THRESHOLD && rotationalSpeed < RELOCALIZATION_ROTATION_THRESHOLD) {
                        for (AprilTagDetection detection : detections) {
                            // Check if this is a valid goal tag (ID 20 for blue, ID 24 for red)
                            if (detection.id == RED_GOAL_TAG_ID || detection.id == BLUE_GOAL_TAG_ID) {
                                // Determine if this is the red or blue goal
                                boolean isRedGoal = (detection.id == RED_GOAL_TAG_ID);

                                // Get AprilTag pose data
                                // ftcPose.x = horizontal offset (right is positive)
                                // ftcPose.y = forward distance (depth)
                                // ftcPose.yaw = tag rotation relative to camera
                                double tagX = detection.ftcPose.x;
                                double tagY = detection.ftcPose.y; // This is the forward distance (depth)
                                double tagYaw = detection.ftcPose.yaw;

                                // Get current turret heading from auto-aim controller
                                double turretHeading = autoAimController.getCurrentTurretHeading();

                                // Calculate robot pose using camera relocalization
                                Pose2D calculatedPose = cameraRelocalization.calculateRobotPose(
                                        tagYaw,
                                        turretHeading,
                                        currentHeadingOdo, // Use current IMU/odo heading
                                        tagX,
                                        tagY,
                                        isRedGoal
                                );

                                telemetry.addData(">> AprilTag Detected: ID ", detection.id);
                                telemetry.update();

                                if (!cameraRelocalization.isPoseValid(calculatedPose)) {
                                    telemetry.addLine(">>> RELOCALIZATION FAILED <<<");
                                    telemetry.addLine("Calculated pose is invalid.");
                                    telemetry.addData("Calculated Pose", "X=%.1f, Y=%.1f, H=%.1f",
                                            calculatedPose.getX(DistanceUnit.INCH),
                                            calculatedPose.getY(DistanceUnit.INCH),
                                            calculatedPose.getHeading(AngleUnit.DEGREES));
                                    telemetry.update();
                                }

                                // Validate and apply the new pose
                                if (calculatedPose != null && cameraRelocalization.isPoseValid(calculatedPose)) {
                                    // Reset odometry to the calculated pose
                                    odo.setPosition(calculatedPose);

                                    // Successfully relocalized - switch to STANDARD mode
                                    currentMode = OperatingMode.STANDARD;

                                    telemetry.addLine(">>> RELOCALIZATION SUCCESSFUL <<<");
                                    telemetry.addData("New Pose", "X=%.1f, Y=%.1f, H=%.1f",
                                            calculatedPose.getX(DistanceUnit.INCH),
                                            calculatedPose.getY(DistanceUnit.INCH),
                                            calculatedPose.getHeading(AngleUnit.DEGREES));
                                    break;
                                }
                            }
                        }
                    } else {
                        // Robot is moving too fast for relocalization
                        telemetry.addData("Relocalization", "Moving too fast! Stop to relocalize.");
                    }
                }
            }

            // ==================== CALCULATIONS ====================
            // Use adjusted pose for auto-aim calculations
            autoAimController.updateRobotPosition(adjustedX, adjustedY, adjustedHeading, autoAimController.getCurrentTurretHeading());
            // Use adjusted pose for distance calculation too
            distanceToGoal = robot.getDistanceFromGoal(adjustedPose, targetIsRed);
            hoodState = robot.getShooterAngle(distanceToGoal);

            // ==================== CONTROLS ====================
            double x, y, rx;

            // Speed toggle (all modes)
            if (gamepad1.a) {
                drivetrainPower = 1.5 - drivetrainPower;
            }

            // ==================== MODE-SPECIFIC CONTROLS ====================
            switch (currentMode) {
                case STANDARD:
                    // Field-centric driving, 1 driver, full auto-aim
                    if (!targetIsRed) {
                        y = gamepad1.left_stick_y;
                        x = -gamepad1.left_stick_x * 1.1;
                    } else {
                        y = -gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x * 1.1;
                    }
                    rx = gamepad1.right_stick_x;

                    // Auto-aim turret using adjusted pose (accounts for movement)
                    // Apply 1.1 multiplier only for long-range shots (>125 inches)
                    double rawTargetAngle = autoAimController.calculateTargetAngle(adjustedX, adjustedY, adjustedHeading);
                    if (distanceToGoal > 125) {
                        calculatedTargetAngle = rawTargetAngle * 1.1;
                    } else {
                        calculatedTargetAngle = rawTargetAngle;
                    }

                    // Dynamic shooter speed based on distance
                    if (gamepad1.right_trigger > 0) {
                        shooterSpeed = robot.getShooterRPM(distanceToGoal);
                    } else {
                        shooter.stopVelocityPIDF();
                        shooter.stopShooter();
                        shooterSpeed = 0;
                    }

                    // Drive field-centric
                    robot.driveFieldCentric(pos, y, x, rx, drivetrainPower);
                    break;

                case RELOCALIZATION:
                    // Field-centric driving while searching for AprilTag
                    if (!targetIsRed) {
                        y = gamepad1.left_stick_y;
                        x = -gamepad1.left_stick_x * 1.1;
                    } else {
                        y = -gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x * 1.1;
                    }
                    rx = gamepad1.right_stick_x;

                    // Keep turret still during relocalization - hold current position
                    calculatedTargetAngle = autoAimController.getCurrentTurretHeading();

                    // Shooter controls same as standard
                    if (gamepad1.right_trigger > 0) {
                        shooterSpeed = robot.getShooterRPM(distanceToGoal);
                    } else {
                        shooter.stopVelocityPIDF();
                        shooter.stopShooter();
                        shooterSpeed = 0;
                    }

                    // Drive field-centric
                    robot.driveFieldCentric(pos, y, x, rx, drivetrainPower);
                    break;

                case MANUAL:
                    // Robot-centric driving, 2 drivers
                    // Driver 2 (gamepad2): Driving and turret
                    y = -gamepad2.left_stick_y;
                    x = gamepad2.left_stick_x * 1.1;
                    rx = gamepad2.right_stick_x;

                    // Driver 2: Turret manual control with limits
                    double turretInput = -gamepad2.left_trigger + gamepad2.right_trigger; // Use triggers for turret
                    if (turretInput != 0) {
                        double currentTurretHeading = autoAimController.getCurrentTurretHeading();
                        manualTurretHeading = currentTurretHeading + (turretInput * 3.0); // Scale for sensitivity
                        // Clamp to limits
                        manualTurretHeading = Math.max(turretLimitCW, Math.min(turretLimitCCW, manualTurretHeading));
                    }

                    // Driver 1 (gamepad1): Shooter presets and intake
                    // Shooter presets (gamepad1)
                    if (gamepad1.a) {
                        // Shotgun preset
                        usingSniperPreset = false;
                    } else if (gamepad1.b) {
                        // Sniper preset
                        usingSniperPreset = true;
                    }

                    // Set shooter speed based on preset (driver 1 controls shooter)
                    if (gamepad1.right_trigger > 0) {
                        if (usingSniperPreset) {
                            shooterSpeed = sniper;
                            hoodState = 0.5;
                        } else {
                            shooterSpeed = shotgunTeleOp;
                            hoodState = 0.42;
                        }
                    } else {
                        shooter.stopVelocityPIDF();
                        shooter.stopShooter();
                        shooterSpeed = 0;
                    }

                    calculatedTargetAngle = manualTurretHeading;

                    // Drive robot-centric (driver 2)
                    robot.driveRobotCentric(y, x, rx, drivetrainPower);
                    break;
            }

            // ==================== COMMON CONTROLS (ALL MODES) ====================
            // Calculate shooter velocity for use in multiple places
            // Use Math.abs() because rightShooter has reversed direction
            double avgShooterVel = (Math.abs(leftShooter.getVelocity()) + Math.abs(rightShooter.getVelocity())) / 2;

            // Intake (driver 1 in all modes)
            // For long-range shots (>125 inches), pulse intake - only run when shooter is at speed
            boolean isLongRangeShot = distanceToGoal > 125;
            boolean shooterAtSpeed = shooterSpeed > 0 && Math.abs(avgShooterVel - shooterSpeed) < shootingToleranceTeleOp;

            if (gamepad1.right_bumper) {
                // If long-range shot, only run intake when shooter is at speed (pulse mode)
                if (isLongRangeShot && shooterSpeed > 0) {
                    intakePower = shooterAtSpeed ? 1 : 0;
                } else {
                    intakePower = 1;
                }
            } else if (gamepad1.left_bumper) {
                intakePower = -0.5;
            } else {
                intakePower = 0;
            }

            // Latch control: only close when flywheel is not running
            if (shooterSpeed > 0) {
                latchState = 1;  // Open latch when shooter is running
            } else {
                latchState = 0;  // Close latch when shooter is off
            }

            // ==================== LIGHT COLOR ====================
            // Base color on mode
            switch (currentMode) {
                case STANDARD:
                    lightState = LIGHT_BLUE;
                    break;
                case RELOCALIZATION:
                    lightState = LIGHT_YELLOW;
                    break;
                case MANUAL:
                    lightState = LIGHT_RED;
                    break;
            }

            // Override with green if shooter is ready
            if (shooterAtSpeed) {
                lightState = LIGHT_GREEN;
            }

            // ==================== SET POWER ====================
            robot.setLatch(latchState);
            hoodServo.setPosition(hoodState);
            if (shooterSpeed != 0) {
                shooter.setVelocityPIDF(shooterSpeed);
            }
            intake.setPower(intakePower);
            turretController.spinToHeadingLoop(calculatedTargetAngle, turretPower);
            light.setPosition(lightState);

            // ==================== TELEMETRY ====================
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("MODE", currentMode.toString());
            telemetry.addData("Alliance", targetIsRed ? "RED" : "BLUE");
            telemetry.addLine("═══════════════════════════════");

            telemetry.addData("Robot Speed", "%.1f", drivetrainPower);
            telemetry.addData("Position", "X=%.1f, Y=%.1f, H=%.1f°", currentXOdo, currentYOdo, currentHeadingOdo);
            telemetry.addData("Distance to Goal", "%.1f in", distanceToGoal);

            telemetry.addLine("--- Turret ---");
            telemetry.addData("Target Heading", "%.1f°", calculatedTargetAngle);
            telemetry.addData("Actual Heading", "%.1f°", autoAimController.getCurrentTurretHeading());

            telemetry.addLine("--- Shooter ---");
            telemetry.addData("Target RPM", "%.0f", shooterSpeed);
            telemetry.addData("Actual RPM", "%.0f", avgShooterVel);
            telemetry.addData("PIDF Debug", shooter.getDebugInfo());
            if (currentMode == OperatingMode.MANUAL) {
                telemetry.addData("Preset", usingSniperPreset ? "SNIPER" : "SHOTGUN");
            }

            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("DPad UP/DOWN to change mode");
            telemetry.update();
        }

        // Cleanup
        if (aprilTagAvailable) {
            AprilTag.close();
        }
        autoAimController.stopAutoAim();
        shooter.stopVelocityPIDF();
        turretController.stopVelocityPID();
        saveEndPosition(0, 0, 0, 0);
    }
}

