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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous
public class AutoBlueShort18Artifacts extends OpMode {

    public Follower follower;
    public ShootingAction shooter;
    public Turret turretControl;
    public RobotActions robot;
    public ShooterController shooterController;
    AutoAim autoAim;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(32, 135, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scoreClosePose = new Pose(56, 76, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scoreFarPose = new Pose(61.5, 69.5, Math.toRadians(180));
    public double shooterVelocity = 0;
    public double hoodAngle = 0;
    public double distanceFromGoal = 0;
    public Pose2D currentPose;
    public double turretAngle = 0;
    int shootDuration = 650; // Duration of the shooting action in milliseconds
    int rampUpDuration = 0; // Duration of the ramp up in milliseconds
    int tolerance = 50; // Tolerance in shooting velocity
    private static final double FARM_MS = 5.0; // Time to farm more from gate before parking, in milliseconds
    private static final int PRE_RAMP_SHOOTER_VELOCITY = 1550; // Shooter velocity to hold between shots in RPM
    private static final int SHOOT_VELOCITY = 1550; // Target shooter velocity in RPM (adjust based on distance)
    private static final int SETTLE_TIME_MS = 100; // Time to wait for Pedro to fully correct position
    private static final int TURRET_AIM_TIME_MS = 100; // Time for turret to aim before shooting
    private static final int LATCH_RELEASE_TIME_MS = 100; // Time to wait after releasing latches before shooting, to allow artifacts to drop
    private static final int GATE_OPEN_MS = 750; // Time for gate opener to open and artifacts to exit
    private static final int INTAKE_ROW_EXTRA_MS = 300; // Additional time to run intake after picking up from row, to ensure artifacts are collected

    // Turret aiming thread - allows turret to update while shooter.shoot() blocks main thread
    private Thread turretAimThread;
    private volatile boolean turretAimRunning = false;
    private volatile double targetTurretAngle = 0;

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
    private Servo light;
    private VoltageSensor voltageSensor;
    public GoBildaPinpointDriver odo;

    public PathChain Shoot1;
    public PathChain Shoot2;
    public PathChain OpenGate;
    public PathChain IntakeGate;
    public PathChain Take2ndRow;
    public PathChain Shoot3;
    public PathChain Take1stRow;
    public PathChain Shoot4;
    public PathChain Take3rdRow;
    public PathChain Shoot5;
    public PathChain Park;

    public void buildPaths() {
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(29.100, 133.500),

                                new Pose(56.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Take2ndRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 82.000),
                                new Pose(53.850, 58.000),// 42,58
                                new Pose(25.000, 60.000) //16, 60
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 60.000),

                                new Pose(56.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        OpenGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 82.000),
                                new Pose(12.500, 63.500) // Old: 11.5, 61.5
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.24))
                .build();

        IntakeGate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 70.000),
                                new Pose(13.125, 63.845)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.500, 61.500),
                                new Pose(56.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143.24), Math.toRadians(180))
                .build();

        Take1stRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 82.000),
                                new Pose(49.000, 84.000),
                                new Pose(25.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.000, 87.000),

                                new Pose(56.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Take3rdRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 82.000),
                                new Pose(50.000, 32.000),
                                new Pose(16.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 36.000),

                                new Pose(56.000, 82.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 82.000),

                                new Pose(60.000, 108.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                /* Shoot 1st burst */
                leftLatch.setPosition(0);
                hoodServo.setPosition(0.24);
                shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                follower.followPath(Shoot1, true);
                turretControl.spinToHeadingLoop(132, turretSpeedAuto);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(101); // Go to settling state
                }
                break;
            case 101:
                // Start turret aiming thread and update target angle
                targetTurretAngle = turretAngle;
                startTurretAimThread();

                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    // Turret thread will keep updating while shoot() blocks
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.24, tolerance, false);
                    stopTurretAimThread();
                    setPathState(2);
                }
                break;
            case 2:
                // Take 2nd row of artifacts
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                    follower.followPath(Take2ndRow, false);
                    setPathState(4);
                }
                break;
            case 4:
                /* Shoot 2nd burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, true);
                    actionTimer.resetTimer();
                    while (actionTimer.getElapsedTime() < INTAKE_ROW_EXTRA_MS) {
                        intake.setPower(1); // Keep intake on for a short time after picking up from row, to ensure artifacts are collected
                    }
                    //turretControl.spinToHeadingLoop(135, turretSpeedAuto);
                    while (actionTimer.getElapsedTime() < (LATCH_RELEASE_TIME_MS + INTAKE_ROW_EXTRA_MS)) {
                        // Wait for latches to release before shooting to prevent jams
                        intake.setPower(0.0); //Stop intake to buy time for latches
                    }
                    leftLatch.setPosition(0);
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
                // Start turret aiming thread and update target angle
                intake.setPower(0.0);
                targetTurretAngle = turretAngle;
                startTurretAimThread();

                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    // Turret thread will keep updating while shoot() blocks
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.24, tolerance, false);
                    stopTurretAimThread();
                    setPathState(6);
                }
                break;
            case 6:
                /* Open Gate */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(OpenGate, true);
                    setPathState(106);
                    //actionTimer.resetTimer();
                }
                break;
            case 206:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeGate, true);
                    setPathState(106);
                    actionTimer.resetTimer();
                }
                break;
            case 106:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > GATE_OPEN_MS) {
                        setPathState(7);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    intake.setPower(0); // Stop intake before moving to shoot
                    leftLatch.setPosition(0); // Open latch
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY);
                    follower.followPath(Shoot3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(108); // Go to settling state
                }
                break;
            case 108:
                // Start turret aiming thread and update target angle
                intake.setPower(0.0);
                targetTurretAngle = turretAngle;
                startTurretAimThread();

                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    // Turret thread will keep updating while shoot() blocks
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.24, tolerance, false);
                    stopTurretAimThread();
                    setPathState(10);
                }
                break;
            case 10:
                /* Take 1st row */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY); // Start ramping up shooter early
                    follower.followPath(Take1stRow, false);
                    setPathState(11);
                }
                break;
            case 11:
                /* Shoot 4th burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, true);
                    actionTimer.resetTimer();
                    while (actionTimer.getElapsedTime() < INTAKE_ROW_EXTRA_MS) {
                        intake.setPower(1); // Keep intake on for a short time after picking up from row
                    }
                    while (actionTimer.getElapsedTime() < (LATCH_RELEASE_TIME_MS + INTAKE_ROW_EXTRA_MS)) {
                        intake.setPower(0.0); // Stop intake to buy time for latches
                    }
                    leftLatch.setPosition(0);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(112); // Go to settling state
                }
                break;
            case 112:
                // Start turret aiming thread and update target angle
                intake.setPower(0.0);
                targetTurretAngle = turretAngle;
                startTurretAimThread();

                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    // Turret thread will keep updating while shoot() blocks
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.24, tolerance, false);
                    stopTurretAimThread();
                    setPathState(13);
                }
                break;
            case 13:
                /* Open Gate for 5th burst */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(OpenGate, true);
                    setPathState(113);
                    //actionTimer.resetTimer();
                }
                break;
            case 213:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeGate, true);
                    setPathState(113);
                    actionTimer.resetTimer();
                }
                break;
            case 113:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTime() > GATE_OPEN_MS) {
                        setPathState(14);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 14:
                // Wait for gate to open and artifacts to exit, then go shoot
                if (!follower.isBusy()) {
                    intake.setPower(0); // Stop intake before moving to shoot
                    leftLatch.setPosition(0); // Open latch
                    shooterController.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY);
                    follower.followPath(Shoot3, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(115); // Go to settling state
                }
                break;
            case 115:
                // Start turret aiming thread and update target angle
                intake.setPower(0.0);
                targetTurretAngle = turretAngle;
                startTurretAimThread();

                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    // Turret thread will keep updating while shoot() blocks
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.33, tolerance, false);
                    stopTurretAimThread();
                    double remainingTime = 30.0 - opmodeTimer.getElapsedTimeSeconds();
                    if (remainingTime < FARM_MS) {
                        setPathState(17); // Go to Park
                    } else {
                        setPathState(13); // Loop back to collect more from gate
                    }
                }
                break;
            case 17:
                /* Park */
                if (!follower.isBusy()) {
                    shooterController.stopShooter();
                    turretControl.spinToHeadingLoop(0, turretSpeedAuto); // Face forward for parking
                    follower.followPath(Park, true);
                    setPathState(18);
                }
                break;
            case 18:
                /* Wait for parking to complete */
                if (!follower.isBusy()) {
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
                break;
        }
    }

    /**
     * Starts the turret aiming thread. Call this when entering a shooting/settling state.
     * The thread will continuously update the turret position based on current robot pose.
     */
    private void startTurretAimThread() {
        if (turretAimRunning) return; // Already running

        turretAimRunning = true;
        turretAimThread = new Thread(() -> {
            while (turretAimRunning && !Thread.currentThread().isInterrupted()) {
                try {
                    // Get fresh robot pose from follower
                    follower.update();
                    Pose currentPose = follower.getPose();
                    double fieldX = currentPose.getX() - 72;
                    double fieldY = currentPose.getY() - 72;
                    double headingDegrees = Math.toDegrees(currentPose.getHeading());

                    // Calculate fresh turret angle based on current position
                    double freshTurretAngle = autoAim.calculateTargetAngle(fieldX, fieldY, headingDegrees) - 2.0; // Add small offset to counter correction
                    targetTurretAngle = freshTurretAngle;

                    // Update turret position
                    turretControl.spinToHeadingLoop(targetTurretAngle, turretSpeedAuto);
                    Thread.sleep(10); // 100Hz update rate
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Ignore other exceptions to keep thread running
                }
            }
        });
        turretAimThread.setDaemon(true);
        turretAimThread.start();
    }

    /**
     * Stops the turret aiming thread. Call this when exiting a shooting state.
     */
    private void stopTurretAimThread() {
        turretAimRunning = false;
        if (turretAimThread != null) {
            turretAimThread.interrupt();
            try {
                turretAimThread.join(100); // Wait up to 100ms for thread to stop
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            turretAimThread = null;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // Safety margin - stop early to ensure position is saved before 30s force stop
    private static final double AUTO_SAFETY_STOP_SECONDS = 29.75;
    private volatile boolean autoStopped = false;

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        // Check if we should stop early to save position
        if (!autoStopped && opmodeTimer.getElapsedTimeSeconds() >= AUTO_SAFETY_STOP_SECONDS) {
            autoStopped = true;

            // Stop Pedro Pathing follower first to get accurate final pose
            try {
                follower.breakFollowing();
            } catch (Exception e) {
                // Ignore
            }

            // Give a moment for the robot to settle
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                // Ignore
            }

            // Update follower one last time to get final position
            try {
                follower.update();
            } catch (Exception e) {
                // Ignore
            }

            // Stop all motors and threads immediately
            try {
                stopTurretAimThread();
                intake.setPower(0);
                shooterController.stopVelocityPIDF();
                turretControl.stopVelocityPID();
            } catch (Exception e) {
                // Ignore
            }

            // Save position now while we still can
            try {
                Pose endPose = follower.getPose();
                org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition(
                        (endPose.getX() - 72),
                        (endPose.getY() - 72),
                        Math.toDegrees(endPose.getHeading()),
                        turretControl.getCurrentHeading()
                );
                telemetry.addLine("═══════════════════════════════");
                telemetry.addLine("   POSITION SAVED SUCCESSFULLY");
                telemetry.addLine("═══════════════════════════════");
                telemetry.update();
            } catch (Exception e) {
                telemetry.addLine(">>> FAILED TO SAVE POSITION <<<");
                telemetry.update();
            }
            return;
        }

        // If already stopped, just show status
        if (autoStopped) {
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("   AUTO COMPLETE - WAITING");
            telemetry.addData("Time", "%.1f s", opmodeTimer.getElapsedTimeSeconds());
            telemetry.addLine("═══════════════════════════════");
            telemetry.update();
            return;
        }

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
            distanceFromGoal = robot.getDistanceFromGoal(currentPose, false);
            shooterVelocity = robot.getShooterRPM(distanceFromGoal);
            hoodAngle = robot.getShooterAngle(distanceFromGoal);
            turretAngle = autoAim.calculateTargetAngle(fieldX, fieldY, headingDegrees);

            // Note: Turret is updated in autonomousPathUpdate() before shooting, not here
            // This allows the turret to settle before the blocking shoot() call

            try {
                autonomousPathUpdate();
            } catch (InterruptedException e) {
                // Handle interruption gracefully - autonomous was force stopped
                return;
            }
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
        targetIsRed = false;

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
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
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
        hoodServo.setDirection(Servo.Direction.REVERSE);
        light = hardwareMap.get(Servo.class, "light");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        autoAim = new AutoAim(turret, telemetry, false);

        shooter = new ShootingAction(
                leftShooter,
                rightShooter,
                intake,
                turret,
                hoodServo,
                leftLatch,
                shooterController,
                turretControl,
                autoAim
        );

        robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake, leftLatch, hoodServo, light);
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
        // Stop turret aiming thread if running
        try {
            stopTurretAimThread();
        } catch (Exception e) {
            // Ignore
        }

        // Ensure PID threads stop when OpMode ends
        try {
            shooterController.stopVelocityPID();
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