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
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp2;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

    // Movement types
    private enum MovementType {
        FIELD_CENTRIC,  // Field-centric driving (default)
        QUAN_TELEOP_V2  // Robot-centric driving (Quan's preference)
    }

    // Light colors for each mode
    private static final double LIGHT_BLUE = 0.611;    // Standard mode
    private static final double LIGHT_YELLOW = 0.388;  // Relocalization mode
    private static final double LIGHT_RED = 0.28;      // Manual mode
    private static final double LIGHT_GREEN = 0.5;     // Shooter ready
    private static final double LIGHT_PINK = 0.72;     // System disconnected/error

    // Relocalization thresholds
    private static final double RELOCALIZATION_SPEED_THRESHOLD = 0.1; // inches/second for linear velocity
    private static final double RELOCALIZATION_ROTATION_THRESHOLD = 0.1; // degrees/second for rotational velocity
    private static final int RED_GOAL_TAG_ID = 24;  // AprilTag ID for red goal
    private static final int BLUE_GOAL_TAG_ID = 20; // AprilTag ID for blue goal

    // Intake full detection
    private static final double INTAKE_FULL_CURRENT_THRESHOLD = 3.0; // Amps - tune via ShooterRegression
    private static final long INTAKE_FULL_DURATION_MS = 300; // Must be above threshold for this long to be considered full
    private static final long LIGHT_FLASH_INTERVAL_MS = 500; // Flash interval in milliseconds

    // Camera relocalization toggle - set to true to enable background relocalization in STANDARD mode
    private static final boolean ENABLE_BACKGROUND_RELOCALIZATION = false;

    // Relocalization method toggle:
    // false = Use calculateRobotPose (uses current turret heading and robot heading from odometry)
    // true = Use calculateRobotPoseAndHeading (sets turret to 0 and calculates heading from AprilTag)
    private static final boolean USE_HEADING_FROM_APRILTAG = false;

    // Shooter idle power - raw power when not actively shooting
    private static final double SHOOTER_IDLE_POWER = 0.5;

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

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        // Note: ShooterController sets direction and run mode

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo light = hardwareMap.get(Servo.class, "light");

        hoodServo.setDirection(Servo.Direction.REVERSE);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Activate External Classes
        ShooterController shooter = new ShooterController(leftShooter, rightShooter,
                shooterKp, shooterKi, shooterKd, shooterKf,
                shooterKp2, shooterKi2, shooterKd2, shooterKf2,
                voltageSensor, telemetry);
        // Initialize turret with heading from autonomous (don't reset encoder)
        Turret turretController = new Turret(turret, telemetry, autoEndTurretHeading);
        RobotActions robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake,
                leftLatch, hoodServo, light);

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
        Pose2D pos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        int latchState = 0;
        double hoodState = 0.5;
        double intakePower = 0;
        double lightState = 0;
        double calculatedTargetAngle = 0;
        double shooterSpeed = 0;

        // Info Variables
        int target = redTagID;
        double currentHeading = 0.0;
        double distanceToGoal = 0;

        // Operating mode
        OperatingMode currentMode = OperatingMode.STANDARD;
        boolean shootingWhileMoving = true;

        // Alliance selection (during init)
        boolean allianceIsRed = targetIsRed;
        ElapsedTime debounceTimer = new ElapsedTime();

        // Odometry - initialize with defaults so robot can still function if odometry fails
        double currentXOdo = 0;
        double currentYOdo = 0;
        double currentHeadingOdo = 0;
        double x_velocity = 0;
        double y_velocity = 0;
        double heading_velocity = 0;

        // Manual mode turret preset tracking
        boolean usingSniperPreset = false;

        // Intake current monitoring and light flashing
        double intakeCurrent = 0;
        boolean intakeFull = false;
        ElapsedTime intakeFullTimer = new ElapsedTime(); // Tracks how long current has been above threshold
        boolean currentAboveThreshold = false; // Tracks if current is currently above threshold
        ElapsedTime flashTimer = new ElapsedTime();
        boolean flashState = false; // false = mode color, true = green

        // System error tracking - turns light pink if any system disconnects
        boolean systemError = false;

        // ==================== INIT PHASE: COMBINED SELECTION ====================
        // DPad UP/DOWN = Alliance selection
        // DPad LEFT/RIGHT = Movement type selection
        // Press START to begin match
        MovementType selectedMovementType = MovementType.FIELD_CENTRIC;

        while (!isStarted() && !isStopRequested()) {
            // Handle selections with debounce
            if (debounceTimer.milliseconds() > 250) {
                // Alliance selection: DPad UP/DOWN
                if (gamepad1.dpad_up) {
                    allianceIsRed = true;
                    debounceTimer.reset();
                } else if (gamepad1.dpad_down) {
                    allianceIsRed = false;
                    debounceTimer.reset();
                }

                // Movement type selection: DPad LEFT/RIGHT
                if (gamepad1.dpad_left) {
                    selectedMovementType = MovementType.FIELD_CENTRIC;
                    debounceTimer.reset();
                } else if (gamepad1.dpad_right) {
                    selectedMovementType = MovementType.QUAN_TELEOP_V2;
                    debounceTimer.reset();
                }
            }

            // Display combined selection screen
            telemetry.addLine("╔══════════════════════════════════════════╗");
            telemetry.addLine("║         TeleOp V3 - Configuration        ║");
            telemetry.addLine("╠══════════════════════════════════════════╣");
            telemetry.addLine("║                                          ║");
            telemetry.addLine("║  ALLIANCE         │  MOVEMENT TYPE       ║");
            telemetry.addLine("║  (DPad UP/DOWN)   │  (DPad LEFT/RIGHT)   ║");
            telemetry.addLine("║                   │                      ║");

            // Alliance display
            String redText = allianceIsRed ? "▶ RED ◀" : "  Red  ";
            String blueText = allianceIsRed ? "  Blue  " : "▶ BLUE ◀";

            // Movement type display
            String fieldText = selectedMovementType == MovementType.FIELD_CENTRIC ? "▶ Field Centric ◀" : "  Field Centric  ";
            String quanText = selectedMovementType == MovementType.QUAN_TELEOP_V2 ? "▶ Quan TeleOp V2 ◀" : "  Quan TeleOp V2  ";

            telemetry.addData("║  ", "%s    │  %s", redText, fieldText);
            telemetry.addData("║  ", "%s   │  %s", blueText, quanText);

            telemetry.addLine("║                                          ║");
            telemetry.addLine("╠══════════════════════════════════════════╣");
            telemetry.addLine("║  DPad UP = Red    │  DPad LEFT = Field   ║");
            telemetry.addLine("║  DPad DOWN = Blue │  DPad RIGHT = Quan   ║");
            telemetry.addLine("╠══════════════════════════════════════════╣");
            telemetry.addLine("║        Press START to begin match        ║");
            telemetry.addLine("╚══════════════════════════════════════════╝");

            telemetry.addLine("");
            if (selectedMovementType == MovementType.QUAN_TELEOP_V2) {
                telemetry.addLine("Quan TeleOp V2: Robot-centric driving");
                telemetry.addLine("Manual mode: Gamepad 1 drives");
            } else {
                telemetry.addLine("Field Centric: Field-oriented driving");
                telemetry.addLine("Manual mode: Gamepad 2 drives");
            }

            telemetry.addData("Auton end position (X, Y, Heading)", "(%.1f, %.1f, %.1f)", autoEndX, autoEndY, autoEndHeading);
            telemetry.addData("Auton end turret heading", "%.1f", autoEndTurretHeading);

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
            try {
                // Reset system error flag at start of each loop
                systemError = false;

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
            // Wrap in try-catch so driving can continue even if odometry fails
            // Declare adjusted pose variables at higher scope for use later
            double adjustedX = currentXOdo;
            double adjustedY = currentYOdo;
            double adjustedHeading = currentHeadingOdo;
            Pose2D adjustedPose = pos; // Default to last known pose
            boolean odoAvailable = true;

            try {
                // Get position
                odo.update();
                Pose2D currentPose = odo.getPosition();
                x_velocity = odo.getVelX(DistanceUnit.INCH);
                y_velocity = odo.getVelY(DistanceUnit.INCH);
                heading_velocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

                // Get shoot-while-moving offsets and create adjusted pose (only in STANDARD mode)
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
                adjustedX = adjustedPose.getX(DistanceUnit.INCH);
                adjustedY = adjustedPose.getY(DistanceUnit.INCH);
                adjustedHeading = adjustedPose.getHeading(AngleUnit.DEGREES);

                // Use currentPose for driving, adjustedPose for auto-aim
                pos = currentPose;
            } catch (Exception e) {
                odoAvailable = false;
                systemError = true;
                telemetry.addLine(">>> ODOMETRY ERROR <<<");
                // Use last known values (already set above)
            }

            // ==================== RELOCALIZATION MODE ====================
            // ==================== CAMERA RELOCALIZATION ====================
            // In STANDARD mode: Run quietly in background to update odometry (if enabled)
            // In RELOCALIZATION mode: Set turret to 0 and use calculateRobotPoseAndHeading
            try {
                if (currentMode == OperatingMode.STANDARD && ENABLE_BACKGROUND_RELOCALIZATION) {
                // Quiet background relocalization - only when nearly stationary
                if (aprilTagAvailable && aprilTagProcessor != null) {
                    double linearSpeed = Math.sqrt(x_velocity * x_velocity + y_velocity * y_velocity);
                    double rotationalSpeed = Math.abs(heading_velocity);

                    if (linearSpeed < RELOCALIZATION_SPEED_THRESHOLD && rotationalSpeed < RELOCALIZATION_ROTATION_THRESHOLD) {
                        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                        for (AprilTagDetection detection : detections) {
                            if (detection.id == RED_GOAL_TAG_ID || detection.id == BLUE_GOAL_TAG_ID) {
                                boolean isRedGoal = (detection.id == RED_GOAL_TAG_ID);
                                double tagX = detection.ftcPose.x;
                                double tagY = detection.ftcPose.y;
                                double turretHeading = autoAimController.getCurrentTurretHeading();

                                Pose2D calculatedPose = cameraRelocalization.calculateRobotPose(
                                        turretHeading,
                                        currentHeadingOdo,
                                        tagX,
                                        tagY,
                                        isRedGoal
                                );

                                // Quietly update odometry if pose is valid
                                if (calculatedPose != null && cameraRelocalization.isPoseValid(calculatedPose)) {
                                    odo.setPosition(calculatedPose);
                                }
                                break;
                            }
                        }
                    }
                }
            } else if (currentMode == OperatingMode.RELOCALIZATION) {
                // Set turret position based on relocalization method
                if (USE_HEADING_FROM_APRILTAG) {
                    // Set turret to 0 (facing backward) for AprilTag-based heading calculation
                    turretController.spinToHeadingLoop(0, turretPower);
                } else {
                    // Lock turret at current position for pose-only relocalization
                    // This keeps the turret stable so we can get accurate readings with known turret heading
                    turretController.spinToHeadingLoop(autoAimController.getCurrentTurretHeading(), turretPower);
                }

                // Turret error adjustment - non-movement gamepad
                // Field Centric: Gamepad 1, Quan TeleOp V2: Gamepad 2
                if (debounceTimer.milliseconds() > 200) {
                    if (selectedMovementType == MovementType.QUAN_TELEOP_V2) {
                        // Gamepad 2 adjusts turret error
                        if (gamepad2.dpad_up) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() + 1);
                            debounceTimer.reset();
                        } else if (gamepad2.dpad_down) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() - 1);
                            debounceTimer.reset();
                        } else if (gamepad2.dpad_right) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() + 5);
                            debounceTimer.reset();
                        } else if (gamepad2.dpad_left) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() - 5);
                            debounceTimer.reset();
                        }
                    } else {
                        // Gamepad 1 adjusts turret error
                        if (gamepad1.dpad_up) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() + 1);
                            debounceTimer.reset();
                        } else if (gamepad1.dpad_down) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() - 1);
                            debounceTimer.reset();
                        } else if (gamepad1.dpad_right) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() + 5);
                            debounceTimer.reset();
                        } else if (gamepad1.dpad_left) {
                            turretController.setEncoderOffset(turretController.getEncoderOffset() - 5);
                            debounceTimer.reset();
                        }
                    }
                }

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
                                double tagX = detection.ftcPose.x;
                                double tagY = detection.ftcPose.y;

                                Pose2D calculatedPose;
                                if (USE_HEADING_FROM_APRILTAG) {
                                    // Use calculateRobotPoseAndHeading since turret is at 0 (facing backward)
                                    // This assumes turret/IMU might be incorrect and calculates heading from AprilTag
                                    calculatedPose = cameraRelocalization.calculateRobotPoseAndHeading(
                                            tagX,
                                            tagY,
                                            isRedGoal
                                    );
                                } else {
                                    // Use calculateRobotPose with current turret heading and robot heading
                                    // This trusts the current turret encoder and IMU/odometry heading
                                    double turretHeading = autoAimController.getCurrentTurretHeading();
                                    calculatedPose = cameraRelocalization.calculateRobotPose(
                                            turretHeading,
                                            currentHeadingOdo,
                                            tagX,
                                            tagY,
                                            isRedGoal
                                    );
                                }

                                telemetry.addData(">> AprilTag Detected: ID ", detection.id);

                                if (!cameraRelocalization.isPoseValid(calculatedPose)) {
                                    telemetry.addLine(">>> RELOCALIZATION FAILED <<<");
                                    telemetry.addLine("Calculated pose is invalid.");
                                    telemetry.addData("Calculated Pose", "X=%.1f, Y=%.1f, H=%.1f",
                                            calculatedPose.getX(DistanceUnit.INCH),
                                            calculatedPose.getY(DistanceUnit.INCH),
                                            calculatedPose.getHeading(AngleUnit.DEGREES));
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
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> CAMERA ERROR <<<");
            }

            // ==================== CALCULATIONS ====================
            // Use adjusted pose for auto-aim calculations
            try {
                autoAimController.updateRobotPosition(adjustedX, adjustedY, adjustedHeading, autoAimController.getCurrentTurretHeading());
                // Use adjusted pose for distance calculation too
                distanceToGoal = robot.getDistanceFromGoal(adjustedPose, targetIsRed);
                hoodState = robot.getShooterAngle(distanceToGoal);
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> AUTO-AIM CALC ERROR <<<");
            }

            // ==================== CONTROLS ====================
            double x, y, rx;

            // Speed toggle (all modes)
            if (gamepad1.a) {
                drivetrainPower = 1.5 - drivetrainPower;
            }

            // ==================== MODE-SPECIFIC CONTROLS ====================
            try {
                switch (currentMode) {
                case STANDARD:
                    // Driving based on selected movement type
                    if (selectedMovementType == MovementType.FIELD_CENTRIC) {
                        // Field-centric driving
                        if (!targetIsRed) {
                            y = gamepad1.left_stick_y;
                            x = -gamepad1.left_stick_x * 1.1;
                        } else {
                            y = -gamepad1.left_stick_y;
                            x = gamepad1.left_stick_x * 1.1;
                        }
                    } else {
                        // Quan TeleOp V2: Robot-centric driving
                        y = -gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x * 1.1;
                    }
                    rx = gamepad1.right_stick_x;

                    // Auto-aim turret using adjusted pose (accounts for movement)
                    // Apply 1.1 multiplier only for long-range shots (>125 inches)
                    double rawTargetAngle = autoAimController.calculateTargetAngle(adjustedX, adjustedY, adjustedHeading);

                    calculatedTargetAngle = rawTargetAngle; // Start with raw angle by default

                    /* Doesn't work
                    if (distanceToGoal > 125) {
                        calculatedTargetAngle = rawTargetAngle * 1.1;
                    } else {
                        calculatedTargetAngle = rawTargetAngle;
                    }

                     */

                    // Dynamic shooter speed based on distance
                    if (gamepad1.right_trigger > 0) {
                        shooterSpeed = robot.getShooterRPM(distanceToGoal);
                    } else {
                        shooter.stopVelocityPIDF();
                        shooter.setRawPower(SHOOTER_IDLE_POWER);
                        shooterSpeed = 0;
                    }

                    // Drive based on selected movement type
                    if (selectedMovementType == MovementType.FIELD_CENTRIC) {
                        robot.driveFieldCentric(pos, y, x, rx, drivetrainPower);
                    } else {
                        robot.driveRobotCentric(y, x, rx, drivetrainPower);
                    }
                    break;

                case RELOCALIZATION:
                    // Driving based on selected movement type while searching for AprilTag
                    if (selectedMovementType == MovementType.FIELD_CENTRIC) {
                        // Field-centric driving
                        if (!targetIsRed) {
                            y = gamepad1.left_stick_y;
                            x = -gamepad1.left_stick_x * 1.1;
                        } else {
                            y = -gamepad1.left_stick_y;
                            x = gamepad1.left_stick_x * 1.1;
                        }
                    } else {
                        // Quan TeleOp V2: Robot-centric driving
                        y = -gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x * 1.1;
                    }
                    rx = gamepad1.right_stick_x;

                    // Set turret target angle based on relocalization method
                    if (USE_HEADING_FROM_APRILTAG) {
                        // Turret is set to 0 in the relocalization section above
                        calculatedTargetAngle = 0;
                    } else {
                        // Turret is locked at current position in the relocalization section above
                        // Just display the current heading (don't auto-aim)
                        calculatedTargetAngle = autoAimController.getCurrentTurretHeading();
                    }

                    // Shooter controls same as standard
                    if (gamepad1.right_trigger > 0) {
                        shooterSpeed = robot.getShooterRPM(distanceToGoal);
                    } else {
                        shooter.stopVelocityPIDF();
                        shooter.setRawPower(SHOOTER_IDLE_POWER);
                        shooterSpeed = 0;
                    }

                    // Drive based on selected movement type
                    if (selectedMovementType == MovementType.FIELD_CENTRIC) {
                        robot.driveFieldCentric(pos, y, x, rx, drivetrainPower);
                    } else {
                        robot.driveRobotCentric(y, x, rx, drivetrainPower);
                    }
                    break;

                case MANUAL:
                    // Manual mode driving depends on movement type selection
                    // Field Centric: Gamepad 2 drives, Gamepad 1 controls shooter
                    // Quan TeleOp V2: Gamepad 1 drives, Gamepad 2 controls turret

                    // TODO: Change corner reset button to driver preference
                    // Corner pose reset for manual mode - resets to corner position as last resort for auto aim
                    // Movement driver presses DPad Left + Back to reset pose to corner
                    // Field Centric: Gamepad 2, Quan TeleOp V2: Gamepad 1
                    boolean cornerResetPressed = selectedMovementType == MovementType.QUAN_TELEOP_V2
                            ? (gamepad1.start)
                            : (gamepad2.start);

                    if (cornerResetPressed) {
                        if (targetIsRed) {
                            // Red corner position
                            odo.setPosition(new Pose2D(DistanceUnit.INCH, -65, -64.5, AngleUnit.DEGREES, 90.0));
                        } else {
                            // Blue corner position
                            odo.setPosition(new Pose2D(DistanceUnit.INCH, 65, -64.5, AngleUnit.DEGREES, 90.0));
                        }
                        // Go back to standard mode after pose reset
                        currentMode = OperatingMode.STANDARD;
                    }

                    // Turret error adjustment - non-movement gamepad (only when back is NOT pressed)
                    // Field Centric: Gamepad 1, Quan TeleOp V2: Gamepad 2
                    // DPad Up/Down: +/- 1 degree, DPad Left/Right: +/- 5 degrees
                    if (debounceTimer.milliseconds() > 200) {
                        if (selectedMovementType == MovementType.QUAN_TELEOP_V2) {
                            // Gamepad 2 adjusts turret error (non-movement gamepad)
                            if (!gamepad2.back) {
                                if (gamepad2.dpad_up) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() + 1);
                                    debounceTimer.reset();
                                } else if (gamepad2.dpad_down) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() - 1);
                                    debounceTimer.reset();
                                } else if (gamepad2.dpad_right) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() + 5);
                                    debounceTimer.reset();
                                } else if (gamepad2.dpad_left) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() - 5);
                                    debounceTimer.reset();
                                }
                            }
                        } else {
                            // Gamepad 1 adjusts turret error (non-movement gamepad)
                            if (!gamepad1.back) {
                                if (gamepad1.dpad_up) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() + 1);
                                    debounceTimer.reset();
                                } else if (gamepad1.dpad_down) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() - 1);
                                    debounceTimer.reset();
                                } else if (gamepad1.dpad_right) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() + 5);
                                    debounceTimer.reset();
                                } else if (gamepad1.dpad_left) {
                                    turretController.setEncoderOffset(turretController.getEncoderOffset() - 5);
                                    debounceTimer.reset();
                                }
                            }
                        }
                    }

                    if (selectedMovementType == MovementType.QUAN_TELEOP_V2) {
                        // Quan TeleOp V2: Gamepad 1 drives (robot-centric)
                        y = -gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x * 1.1;
                        rx = gamepad1.right_stick_x;

                        // Gamepad 2: Turret manual control with limits
                        double turretInput = -gamepad2.left_trigger + gamepad2.right_trigger;
                        if (turretInput != 0) {
                            double currentTurretHeading = autoAimController.getCurrentTurretHeading();
                            manualTurretHeading = currentTurretHeading + (turretInput * 3.0);
                            manualTurretHeading = Math.max(turretLimitCW, Math.min(turretLimitCCW, manualTurretHeading));
                        }

                        // Gamepad 2: Shooter presets
                        if (gamepad2.a) {
                            usingSniperPreset = false; // Shotgun
                        } else if (gamepad2.b) {
                            usingSniperPreset = true; // Sniper
                        }

                        // Gamepad 1: Shooter trigger (since they're driving)
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
                            shooter.setRawPower(SHOOTER_IDLE_POWER);
                            shooterSpeed = 0;
                        }
                    } else {
                        // Field Centric: Gamepad 2 drives (robot-centric in manual mode)
                        y = -gamepad2.left_stick_y;
                        x = gamepad2.left_stick_x * 1.1;
                        rx = gamepad2.right_stick_x;

                        // Gamepad 2: Turret manual control with limits
                        double turretInput = -gamepad2.left_trigger + gamepad2.right_trigger;
                        if (turretInput != 0) {
                            double currentTurretHeading = autoAimController.getCurrentTurretHeading();
                            manualTurretHeading = currentTurretHeading + (turretInput * 3.0);
                            manualTurretHeading = Math.max(turretLimitCW, Math.min(turretLimitCCW, manualTurretHeading));
                        }

                        // Gamepad 1: Shooter presets
                        if (gamepad1.a) {
                            usingSniperPreset = false; // Shotgun
                        } else if (gamepad1.b) {
                            usingSniperPreset = true; // Sniper
                        }

                        // Gamepad 1: Shooter trigger
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
                            shooter.setRawPower(SHOOTER_IDLE_POWER);
                            shooterSpeed = 0;
                        }
                    }

                    calculatedTargetAngle = manualTurretHeading;

                    // Drive robot-centric in manual mode
                    robot.driveRobotCentric(y, x, rx, drivetrainPower);
                    break;
            }
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> DRIVING/CONTROLS ERROR <<<");
            }

            // ==================== COMMON CONTROLS (ALL MODES) ====================
            // Calculate shooter velocity for use in multiple places
            // Use Math.abs() because rightShooter has reversed direction
            double avgShooterVel = 0;
            boolean shooterAtSpeed = false;
            try {
                avgShooterVel = (Math.abs(leftShooter.getVelocity()) + Math.abs(rightShooter.getVelocity())) / 2;
                shooterAtSpeed = shooterSpeed > 0 && Math.abs(avgShooterVel - shooterSpeed) < shootingToleranceTeleOp;
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> SHOOTER VELOCITY READ ERROR <<<");
            }

            // Intake (driver 1 in all modes)
            // For long-range shots (>125 inches), pulse intake - only run when shooter is at speed
            boolean isLongRangeShot = distanceToGoal > 125;

            if (gamepad1.right_bumper) {
                // If long-range shot, only run intake when shooter is at speed (pulse mode)
                if (isLongRangeShot && shooterSpeed > 0) {
                    intakePower = 0.6; // Base power for pulsing
                } else {
                    intakePower = 1; // Full power when not pulsing
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
            // Monitor intake current with consistent detection
            try {
                intakeCurrent = intake.getCurrent(CurrentUnit.AMPS);

                // Check if current is above threshold while intake is running
                boolean isAboveThreshold = intakeCurrent > INTAKE_FULL_CURRENT_THRESHOLD && intake.getPower() > 0;

                if (isAboveThreshold) {
                    // If we just went above threshold, start the timer
                    if (!currentAboveThreshold) {
                        intakeFullTimer.reset();
                        currentAboveThreshold = true;
                    }
                    // Check if we've been above threshold long enough
                    intakeFull = intakeFullTimer.milliseconds() >= INTAKE_FULL_DURATION_MS;
                } else {
                    // Current dropped below threshold, reset
                    currentAboveThreshold = false;
                    intakeFull = false;
                }
            } catch (Exception e) {
                // If intake current reading fails, just disable the full detection
                intakeFull = false;
            }

            // Handle flash timing
            if (flashTimer.milliseconds() > LIGHT_FLASH_INTERVAL_MS) {
                flashState = !flashState;
                flashTimer.reset();
            }

            // Determine base mode color
            double modeColor;
            switch (currentMode) {
                case STANDARD:
                    modeColor = LIGHT_BLUE;
                    break;
                case RELOCALIZATION:
                    modeColor = LIGHT_YELLOW;
                    break;
                case MANUAL:
                default:
                    modeColor = LIGHT_RED;
                    break;
            }

            // Determine final light state
            if (systemError) {
                // System error - solid pink (highest priority, visible from far away)
                lightState = LIGHT_PINK;
            } else if (shooterAtSpeed) {
                // Shooter ready - solid green (high priority)
                lightState = LIGHT_GREEN;
            } else if (intakeFull) {
                // Intake full - flash between mode color and green
                lightState = flashState ? LIGHT_GREEN : modeColor;
            } else {
                // Normal operation - solid mode color
                lightState = modeColor;
            }

            // Set light - in its own try-catch
            try {
                light.setPosition(lightState);
            } catch (Exception e) {
                // Can't even set the light, nothing we can do
            }

            // ==================== SET POWER ====================
            // Wrap each subsystem in try-catch so partial functionality is maintained
            // if one system disconnects

            // Latch
            try {
                robot.setLatch(latchState);
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> LATCH ERROR <<<");
            }

            // Hood servo
            try {
                hoodServo.setPosition(hoodState);
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> HOOD ERROR <<<");
            }

            // Shooter
            try {
                if (shooterSpeed != 0) {
                    shooter.setVelocityPIDF(shooterSpeed);
                }
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> SHOOTER ERROR <<<");
            }

            // Intake
            try {
                intake.setPower(intakePower);
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> INTAKE ERROR <<<");
            }

            // Turret
            try {
                turretController.setRobotAngularVelocity(heading_velocity);
                turretController.spinToHeadingLoop(calculatedTargetAngle, turretPower);
            } catch (Exception e) {
                systemError = true;
                telemetry.addLine(">>> TURRET ERROR <<<");
            }

            // Light - handled separately after determining color

            // ==================== TELEMETRY ====================
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("MODE", currentMode.toString());
            telemetry.addData("Movement", selectedMovementType == MovementType.FIELD_CENTRIC ? "Field Centric" : "Quan TeleOp");
            telemetry.addData("Alliance", targetIsRed ? "RED" : "BLUE");
            telemetry.addLine("═══════════════════════════════");

            telemetry.addData("Robot Speed", "%.1f", drivetrainPower);
            telemetry.addData("Position", "X=%.1f, Y=%.1f, H=%.1f°", currentXOdo, currentYOdo, currentHeadingOdo);
            telemetry.addData("Distance to Goal", "%.1f in", distanceToGoal);

            telemetry.addLine("--- Turret ---");
            telemetry.addData("Target Heading", "%.1f°", calculatedTargetAngle);
            telemetry.addData("Actual Heading", "%.1f°", autoAimController.getCurrentTurretHeading());
            telemetry.addData("Encoder Offset", "%.1f°", turretController.getEncoderOffset());

            telemetry.addLine("--- Shooter ---");
            telemetry.addData("Target RPM", "%.0f", shooterSpeed);
            telemetry.addData("Actual RPM", "%.0f", avgShooterVel);
            telemetry.addData("PIDF Debug", shooter.getDebugInfo());
            if (currentMode == OperatingMode.MANUAL) {
                telemetry.addData("Preset", usingSniperPreset ? "SNIPER" : "SHOTGUN");
            }

            telemetry.addLine("--- Intake ---");
            telemetry.addData("Current", "%.2f A", intakeCurrent);
            telemetry.addData("Intake Full", intakeFull ? ">>> YES <<<" : "No");

            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("DPad UP/DOWN to change mode");
            telemetry.update();

            } catch (Exception e) {
                // Handle hardware disconnection gracefully
                telemetry.addLine(">>> HARDWARE ERROR <<<");
                telemetry.addData("Error", e.getMessage());
                telemetry.addLine("Expansion hub may have disconnected.");
                telemetry.addLine("Attempting to continue...");
                telemetry.update();

                // Small delay to prevent spam
                sleep(100);
            }
        }

        // Cleanup
        try {
            if (aprilTagAvailable) {
                AprilTag.close();
            }
        } catch (Exception e) {
            // Ignore - camera may already be closed
        }

        try {
            autoAimController.stopAutoAim();
        } catch (Exception e) {
            // Ignore - hardware may be disconnected
        }

        try {
            shooter.stopVelocityPIDF();
        } catch (Exception e) {
            // Ignore - hardware may be disconnected
        }

        try {
            turretController.stopVelocityPID();
        } catch (Exception e) {
            // Ignore - hardware may be disconnected
        }

        saveEndPosition(0, 0, 0, 0);
    }
}

