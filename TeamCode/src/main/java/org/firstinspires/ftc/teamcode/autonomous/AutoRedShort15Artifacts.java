package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.targetIsRed;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretSpeedAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.teamcode.lib.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoRedShort15Artifacts extends OpMode {

    public Follower follower;
    public ShootingAction shooter;
    public Turret turretControl;
    public RobotActions robot;
    public ShooterController shooterController;
    AutoAim autoAim;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(112, 133.5, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scoreClosePose = new Pose(56, 76, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scoreFarPose = new Pose(61.5, 69.5, Math.toRadians(180));
    public double shooterVelocity = 0;
    public double hoodAngle = 0;
    public double distanceFromGoal = 0;
    public Pose2D currentPose;
    public double turretAngle = 0;
    int shootDuration = 750; // Duration of the shooting action in milliseconds
    int rampUpDuration = 0; // Duration of the ramp up in milliseconds
    int tolerance = 50; // Tolerance in shooting velocity
    double setHoodAngle = 0.45; // Hood angle for shooting, adjust based on distance
    // TODO: Changes here should go to Blue Short Auto as well
    private static final int PRE_RAMP_SHOOTER_VELOCITY = 1600; // Shooter velocity to hold between shots in RPM
    private static final int SHOOT_VELOCITY = 1600; // Target shooter velocity in RPM (adjust based on distance)
    private static final int SETTLE_TIME_MS = 600; // Time to wait for Pedro to fully correct position
    private static final int TURRET_AIM_TIME_MS = 200; // Time for turret to aim before shooting
    private static final int LATCH_RELEASE_TIME_MS = 100; // Time to wait after releasing latches before shooting, to allow artifacts to drop
    private static final int OPEN_GATE_MS = 0; // Time to wait for gate to open before trying to shoot through it
    private static final int INTAKE_GATE_MS = 500; // Time to run intake when picking up from gate
    private static final int INTAKE_GATE_MS_EXTRA = 2000; // Additional time to run intake after path finishes, to ensure artifacts are collected
    private static final int INTAKE_ROW_EXTRA_MS = 670; // Additional time to run intake after picking up from row, to ensure artifacts are collected
    // Hardware devices
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private DcMotorEx turret;
    private Servo hoodServo;
    private Servo leftLatch;
    private Servo rightLatch;
    private Servo light;
    private VoltageSensor voltageSensor;
    public GoBildaPinpointDriver odo;

    public PathChain Shoot1;
    public PathChain Shoot2;
    public PathChain OpenGate1;
    public PathChain Take2ndRow;
    public PathChain IntakeGate1;
    public PathChain Shoot3;
    public PathChain ShootGateEnd;
    public PathChain Take1stRow;
    public PathChain Shoot4;
    public PathChain Take3rdRow;
    public PathChain Shoot5;
    public PathChain Park;

    public void buildPaths() {
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(112.000, 133.500),

                                new Pose(88.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-10))

                .build();

        Take2ndRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                            new Pose(88.000, 82.000),
                            new Pose(90.150, 58.000),// 42, 58
                            new Pose(119.000, 60.000)// 16, 60
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(0))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 60.000),

                                new Pose(88.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-10))

                .build();

        OpenGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 82.000),
                                new Pose(90.000, 65.000),
                                new Pose(124.000, 65.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(90))

                .build();

        IntakeGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.000, 65.000),
                                new Pose(124.000, 53.000),
                                new Pose(132.000, 47.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 47.000),
                                new Pose(130.000, 76.000),
                                new Pose(107.000, 54.000),
                                new Pose(88.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-10))

                .build();

        ShootGateEnd = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 35.000),
                                new Pose(132.000, 82.500),
                                new Pose(107.000, 54.000),
                                new Pose(84.000, 108.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))

                .build();

        Take1stRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 82.000),
                                new Pose(95.000, 84.000),
                                new Pose(117.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(0))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.000, 87.000),

                                new Pose(88.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-10))

                .build();

        Take3rdRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 82.000),
                                new Pose(94.000, 32.000),
                                new Pose(128.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(0))

                .build();

        Shoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 36.000),

                                new Pose(88.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-10))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.000, 82.000),

                                new Pose(84.000, 108.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(90))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /* Shoot 1st burst */
                leftLatch.setPosition(0);
                rightLatch.setPosition(0);
                hoodServo.setPosition(setHoodAngle);
                shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                follower.followPath(Shoot1, true);
                turretControl.spinToHeadingLoop(-132, turretSpeedAuto);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(101); // Go to settling state
                }
                break;
            case 101:
                // Update turret aim, then wait for it to settle before shooting
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, setHoodAngle, tolerance, false);
                    setPathState(2);
                }
                break;
            case 2:
                // Take 2nd row of artifacts
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                    follower.followPath(Take2ndRow, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* Shoot 2nd burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, true);
                    actionTimer.resetTimer();
                    setPathState(104); // Go to intake extra time state
                }
                break;
            case 104:
                // Run intake for INTAKE_ROW_EXTRA_MS, then stop and wait for latch release
                if (actionTimer.getElapsedTime() < INTAKE_ROW_EXTRA_MS) {
                    intake.setPower(1); // Keep intake on for a short time after picking up from row
                } else if (actionTimer.getElapsedTime() < (LATCH_RELEASE_TIME_MS + INTAKE_ROW_EXTRA_MS)) {
                    intake.setPower(0.0); // Stop intake to buy time for latches
                } else {
                    leftLatch.setPosition(0);
                    rightLatch.setPosition(0);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(105); // Go to settling state
                }
                break;
            case 105:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, setHoodAngle, tolerance, false);
                    setPathState(6);
                }
                break;
            case 6:
                /* Open Gate */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(OpenGate1, true);
                    setPathState(7);
                    actionTimer.resetTimer();
                }
                break;
            case 7:
                // Let artifacts exit gate
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > OPEN_GATE_MS) {
                        setPathState(8);
                    }
                } else {
                    actionTimer.resetTimer(); // Reset timer if still busy to ensure full 0.3 seconds after path finishes
                }
                break;
            case 8:
                /* Take from gate */
                if (!follower.isBusy()) {
                    follower.followPath(IntakeGate1, true);
                    setPathState(9);
                    actionTimer.resetTimer();
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                    if (actionTimer.getElapsedTime() > INTAKE_GATE_MS) {
                        setPathState(10);
                    }
                } else {
                    actionTimer.resetTimer(); // Reset timer if still busy to ensure full 2 seconds after path finishes
                }
                break;
            case 10:
                /* Shoot 3rd burst */
                if (!follower.isBusy()) {
                    //turretControl.spinToHeadingLoop(138, turretSpeedAuto);
                    follower.followPath(Shoot3, true);
                    actionTimer.resetTimer();
                    setPathState(110);
                }
                break;
            case 110:
                if (actionTimer.getElapsedTime() > INTAKE_GATE_MS_EXTRA + LATCH_RELEASE_TIME_MS) {
                    leftLatch.setPosition(0);
                    rightLatch.setPosition(0);
                    setPathState(11);
                } else if (actionTimer.getElapsedTime() > INTAKE_GATE_MS_EXTRA) {
                    intake.setPower(0.0); //Stop intake to buy time for latches
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(111); // Go to settling state
                }
                break;
            case 111:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, setHoodAngle, tolerance, false);
                    setPathState(12);
                }
                break;
            case 12:
                /* Take 1st row */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                    follower.followPath(Take1stRow, true);
                    setPathState(13);
                }
                break;
            case 13:
                /* Shoot 4th burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, true);
                    actionTimer.resetTimer();
                    setPathState(113); // Go to intake extra time state
                }
                break;
            case 113:
                // Run intake for INTAKE_ROW_EXTRA_MS, then stop and wait for latch release
                if (actionTimer.getElapsedTime() < INTAKE_ROW_EXTRA_MS) {
                    intake.setPower(1); // Keep intake on for a short time after picking up from row
                } else if (actionTimer.getElapsedTime() < (LATCH_RELEASE_TIME_MS + INTAKE_ROW_EXTRA_MS)) {
                    intake.setPower(0.0); // Stop intake to buy time for latches
                } else {
                    leftLatch.setPosition(0);
                    rightLatch.setPosition(0);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(114); // Go to settling state
                }
                break;
            case 114:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, setHoodAngle, tolerance, false);
                    setPathState(15);
                }
                break;
            case 15:
                /* Grab from gate */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(OpenGate1, true);
                    setPathState(16);
                    actionTimer.resetTimer();
                }
                break;
            case 16:
                // Let artifacts exit gate
                if (!follower.isBusy()) {
                     if (actionTimer.getElapsedTime() > OPEN_GATE_MS) {
                        setPathState(17);
                    }
                } else {
                    actionTimer.resetTimer(); // Reset timer if still busy to ensure full 1.5 seconds after path finishes
                }
                break;
            case 17:
                /* Collect From Gate */
                if (!follower.isBusy()) {
                    follower.followPath(IntakeGate1, true);
                    setPathState(18);
                    actionTimer.resetTimer();
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    shooterController.setVelocityPIDF(1550); // Start ramping up shooter early
                    if (actionTimer.getElapsedTime() > INTAKE_GATE_MS) {
                        setPathState(19);
                    }
                } else {
                    actionTimer.resetTimer(); // Reset timer if still busy to ensure full 2 seconds after path finishes
                }
                break;
            case 19:
                /* Shoot 5th Burst */
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(-60, turretSpeedAuto);
                    follower.followPath(ShootGateEnd, true);
                    actionTimer.resetTimer();
                    setPathState(119);
                }
                break;
            case 119:
                if (actionTimer.getElapsedTime() > INTAKE_GATE_MS_EXTRA + LATCH_RELEASE_TIME_MS) {
                    leftLatch.setPosition(0);
                    rightLatch.setPosition(0);
                    setPathState(20);
                } else if (actionTimer.getElapsedTime() > INTAKE_GATE_MS_EXTRA) {
                    intake.setPower(0.0); //Stop intake to buy time for latches
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(120); // Go to settling state
                }
                break;
            case 120:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(1550, shootDuration, rampUpDuration, 0.4, tolerance, false);
                    double remainingTime = 30.0 - opmodeTimer.getElapsedTimeSeconds();
                    if (remainingTime < 6.5) {
                        setPathState(-1); // Go to Park
                    } else {
                        setPathState(-1); // Loop back to collect more overflow
                    }
                }
                break;
            case 21:
                /* Park */
                if (!follower.isBusy()) {
                    shooterController.stopShooter();
                    turretControl.spinToHeadingLoop(0, turretSpeedAuto); // Face forward for parking
                    follower.followPath(Park, true);
                    setPathState(22);
                }
                break;
            case 22:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    /* Ideally wait until and of auto so odometry can track collisions */
                    setPathState(-1);
                }
                break;
            case -1:
                // Auto complete - display remaining time
                double elapsedSeconds = opmodeTimer.getElapsedTimeSeconds();
                double remainingSeconds = 30.0 - elapsedSeconds; // FTC Autonomous is 30 seconds
                telemetry.addLine("═══════════════════════════════");
                telemetry.addLine("       AUTO COMPLETE!");
                telemetry.addData("Time Remaining", "%.1f seconds", Math.max(0, remainingSeconds));
                telemetry.addData("Total Time", "%.1f seconds", elapsedSeconds);
                telemetry.addLine("═══════════════════════════════");
                //stop();
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        try {
            // Update follower and get current pose
            follower.update();
            // These loop the movements of the robot, these must be called continuously in order to work
            // Convert Pedro Pathing pose to field coordinates (Pedro origin is at corner, field origin is at center)
            double fieldX = follower.getPose().getX() - 72;
            double fieldY = follower.getPose().getY() - 72;
            double headingRadians = follower.getPose().getHeading();
            double headingDegrees = Math.toDegrees(headingRadians);

            currentPose = new Pose2D(DistanceUnit.INCH, fieldX, fieldY, AngleUnit.DEGREES, headingDegrees);
            distanceFromGoal = robot.getDistanceFromGoal(currentPose, true);
            shooterVelocity = robot.getShooterRPM(distanceFromGoal);
            hoodAngle = robot.getShooterAngle(distanceFromGoal);
            turretAngle = autoAim.calculateTargetAngle(fieldX, fieldY, headingDegrees);

            // Note: Turret is updated in autonomousPathUpdate() before shooting, not here
            // This allows the turret to settle before the blocking shoot() call

            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("field X", "%.1f", fieldX);
            telemetry.addData("field Y", "%.1f", fieldY);
            telemetry.addData("heading (deg)", "%.1f", headingDegrees);
            telemetry.addData("distance to goal", "%.1f", distanceFromGoal);
            telemetry.addData("turret target", "%.1f", turretAngle);
            telemetry.addData("turret current", "%.1f", turretControl.getCurrentHeading());
            telemetry.addData("turret error", "%.1f", turretAngle - turretControl.getCurrentHeading());
            telemetry.update();
        } catch (Exception e) {
            // Handle hardware disconnection or force stop gracefully
            telemetry.addLine(">>> AUTO INTERRUPTED <<<");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        targetIsRed = true;

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterController = new ShooterController(leftShooter, rightShooter,
                shooterKp, shooterKi, shooterKd, shooterKf,
                shooterKp2, shooterKi2, shooterKd2, shooterKf2,
                voltageSensor, telemetry);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretControl = new Turret(turret, telemetry);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        rightLatch.setDirection(Servo.Direction.REVERSE);
        hoodServo.setDirection(Servo.Direction.REVERSE);
        light = hardwareMap.get(Servo.class, "light");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        autoAim = new AutoAim(turret, telemetry, true);

        shooter = new ShootingAction(
                leftShooter,
                rightShooter,
                intake,
                turret,
                hoodServo,
                leftLatch,
                rightLatch,
                shooterController,
                turretControl,
                autoAim
        );

        robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake, leftLatch, rightLatch, hoodServo, light);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * Kill shooter and turret threads and save end position for TeleOp
     **/
    @Override
    public void stop() {
        // FIRST: Signal any in-progress shoot() to bail out immediately
        try {
            shooter.requestStop();
        } catch (Exception e) {
            // Ignore
        }

        // Ensure PID threads stop when OpMode ends
        try {
            shooterController.stopVelocityPIDF();
        } catch (Exception e) {
            // Ignore - hardware may be disconnected
        }

        try {
            turretControl.stopVelocityPID();
        } catch (Exception e) {
            // Ignore - hardware may be disconnected
        }

        // Stop all motors safely
        try {
            intake.setPower(0);
        } catch (Exception e) {
            // Ignore
        }

        // Save final position even on manual termination
        try {
            follower.update();
            Pose endPose = follower.getPose();
            org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition(
                    (endPose.getX() - 72),
                    (endPose.getY() - 72),
                    Math.toDegrees(endPose.getHeading()),
                    turretControl.getCurrentHeading()
            );
        } catch (Exception e) {
            // Ignore - position save is optional
        }
    }
}