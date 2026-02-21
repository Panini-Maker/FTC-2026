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
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.RobotActions;
import org.firstinspires.ftc.teamcode.lib.ShooterController;
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.teamcode.lib.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous

public class AutoRedLong15Artifacts extends OpMode {

    public Follower follower;
    public ShootingAction shooter;
    public Turret turretControl;
    public RobotActions robot;
    public ShooterController controller;
    AutoAim autoAim;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(80.5, 6, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scoreClosePose = new Pose(56, 76, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scoreFarPose = new Pose(61.5, 69.5, Math.toRadians(180));
    public double shooterVelocity = 0;
    public double hoodAngle = 0;
    public double distanceFromGoal = 0;
    public Pose2D currentPose;
    public double turretAngle = 0;
    public int shootDuration = 1450;
    public int rampUpDuration = 0;
    int tolerance = 50; // Tolerance in shooting velocity
    // TODO: Changes here should go to Blue Long Auto as well
    private static final int PRE_RAMP_SHOOTER_VELOCITY = 1850; // Shooter velocity to hold between shots in RPM
    private static final int SHOOT_VELOCITY = 1850; // Target shooter velocity in RPM (adjust based on distance)
    private static final int SETTLE_TIME_MS = 600; // Time to wait for Pedro to fully correct position
    private static final int TURRET_AIM_TIME_MS = 200; // Time for turret to aim before shooting
    private static final int LATCH_RELEASE_TIME_MS = 100; // Time to wait after releasing latches before shooting
    private static final int INTAKE_EXTRA_MS = 200; // Additional time to run intake after path finishes
    private static final int OVERFLOW_COLLECT_TIME_MS = 500; // Time to wait while collecting overflow before shooting


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

    public PathChain Shoot1;
    public PathChain BackUpHumanPlayer;
    public PathChain CollectLastArtifact;
    public PathChain TakeHumanArtifacts;
    public PathChain Shoot2;
    public PathChain Collect3rdRow;
    public PathChain Shoot3;
    public PathChain CollectOverflow;
    public PathChain ShootOverflow;
    public PathChain Park;

    public void buildPaths() {
        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(80.500, 6.000),

                                new Pose(84.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        TakeHumanArtifacts = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 12.000),

                                new Pose(135.000, 13.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        BackUpHumanPlayer = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(135.000, 13.000),
                                new Pose(128.000, 12.000),
                                new Pose(128.000, 7.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        CollectLastArtifact = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 7.000),

                                new Pose(135.000, 7.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.000, 7.000),

                                new Pose(84.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Collect3rdRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 12.000),
                                new Pose(84.000, 36.000),
                                new Pose(131.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.000, 36.000),

                                new Pose(84.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        CollectOverflow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 12.000),
                                //new Pose(131.000, 12.000),
                                new Pose(135.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        ShootOverflow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.000, 12.000),

                                new Pose(84.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 12.000),

                                new Pose(108.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                /* Shoot 1st burst */
                leftLatch.setPosition(0);
                hoodServo.setPosition(0.3); // Hood in advance
                controller.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY);
                follower.followPath(Shoot1, true);
                turretControl.spinToHeadingLoop(-110, turretSpeedAuto);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(101);
                }
                break;
            case 101:
                // Update turret aim, then wait for it to settle before shooting
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.3, tolerance, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* Collect from Human Player */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(TakeHumanArtifacts, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* Back up from Human Player */
                if (!follower.isBusy()) {
                    follower.followPath(BackUpHumanPlayer, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* Collect Last Artifact */
                if (!follower.isBusy()) {
                    follower.followPath(CollectLastArtifact, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* Shoot Second Burst - start moving back while handling intake/latch */
                if (!follower.isBusy()) {
                    controller.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY);
                    follower.followPath(Shoot2, true);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                /* Handle intake/latch while moving back to shoot position */
                if (actionTimer.getElapsedTime() < INTAKE_EXTRA_MS) {
                    intake.setPower(1); // Keep intake on for a short time
                } else {
                    intake.setPower(0.0); // Stop intake
                    if (actionTimer.getElapsedTime() > (LATCH_RELEASE_TIME_MS + INTAKE_EXTRA_MS)) {
                        leftLatch.setPosition(0);
                        // Now wait for path to finish
                        if (!follower.isBusy()) {
                            actionTimer.resetTimer();
                            setPathState(106);
                        }
                    }
                }
                break;
            case 106:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.3, tolerance, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* Collect Near Row */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(Collect3rdRow, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* Shoot 3rd Burst - start moving back while handling intake/latch */
                if (!follower.isBusy()) {
                    controller.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY);
                    follower.followPath(Shoot3, true);
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                /* Handle intake/latch while moving back to shoot position */
                if (actionTimer.getElapsedTime() < INTAKE_EXTRA_MS) {
                    intake.setPower(1); // Keep intake on for a short time
                } else {
                    intake.setPower(0.0); // Stop intake
                    if (actionTimer.getElapsedTime() > (LATCH_RELEASE_TIME_MS + INTAKE_EXTRA_MS)) {
                        leftLatch.setPosition(0);
                        // Now wait for path to finish
                        if (!follower.isBusy()) {
                            actionTimer.resetTimer();
                            setPathState(109);
                        }
                    }
                }
                break;
            case 109:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.3, tolerance, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* Collect Overflow */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(CollectOverflow, true);
                    setPathState(110);
                }
                break;
            case 110:
                /* Wait for overflow collection */
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(111);
                }
                break;
            case 111:
                /* Wait while collecting overflow */
                if (actionTimer.getElapsedTime() > OVERFLOW_COLLECT_TIME_MS) {
                    setPathState(11);
                }
                break;
            case 11:
                /* Shoot Overflow - start moving back while handling intake/latch */
                if (!follower.isBusy()) {
                    controller.setVelocityPIDF(PRE_RAMP_SHOOTER_VELOCITY);
                    follower.followPath(ShootOverflow, true);
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                /* Handle intake/latch while moving back to shoot position */
                if (actionTimer.getElapsedTime() < INTAKE_EXTRA_MS) {
                    intake.setPower(1); // Keep intake on for a short time
                } else {
                    intake.setPower(0.0); // Stop intake
                    if (actionTimer.getElapsedTime() > (LATCH_RELEASE_TIME_MS + INTAKE_EXTRA_MS)) {
                        leftLatch.setPosition(0);
                        // Now wait for path to finish
                        if (!follower.isBusy()) {
                            actionTimer.resetTimer();
                            setPathState(112);
                        }
                    }
                }
                break;
            case 112:
                // Update turret aim, then wait for it to settle before shooting
                intake.setPower(0.0);
                turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                if (actionTimer.getElapsedTime() > SETTLE_TIME_MS + TURRET_AIM_TIME_MS) {
                    shooter.shoot(SHOOT_VELOCITY, shootDuration, rampUpDuration, 0.3, tolerance, true);
                    // Check remaining time - if less than 4 seconds, park. Otherwise, loop back to collect overflow
                    double remainingTime = 30.0 - opmodeTimer.getElapsedTimeSeconds();
                    if (remainingTime < 5.0) {
                        setPathState(13); // Go to Park
                    } else {
                        setPathState(10); // Loop back to collect more overflow
                    }
                }
                break;
            case 13:
                /* Park */
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(0, turretSpeedAuto); // Point turret forward for parking
                    follower.followPath(Park, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            case -1:
                double elapsedSeconds = opmodeTimer.getElapsedTimeSeconds();
                double remainingSeconds = 30.0 - elapsedSeconds;
                telemetry.addLine("═══════════════════════════════");
                telemetry.addLine("       AUTO COMPLETE!");
                telemetry.addData("Time Remaining", "%.1f seconds", Math.max(0, remainingSeconds));
                telemetry.addData("Total Time", "%.1f seconds", elapsedSeconds);
                telemetry.addLine("═══════════════════════════════");
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

            // Stop all motors immediately
            try {
                intake.setPower(0);
                controller.stopVelocityPIDF();
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
            // These loop the movements of the robot, these must be called continuously in order to work
            currentPose = new Pose2D(DistanceUnit.INCH, (follower.getPose().getX() - 72), (follower.getPose().getY() - 72), AngleUnit.RADIANS, follower.getPose().getHeading());
            distanceFromGoal = robot.getDistanceFromGoal(currentPose, true);
            shooterVelocity = robot.getShooterRPM(distanceFromGoal);
            hoodAngle = robot.getShooterAngle(distanceFromGoal);
            turretAngle = autoAim.calculateTargetAngle(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.DEGREES));

            follower.update();
            try {
                autonomousPathUpdate();
            } catch (InterruptedException e) {
                // Handle interruption gracefully - autonomous was force stopped
                return;
            }
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
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

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("intake");
        intake = hardwareMap.dcMotor.get("intake");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new ShooterController(leftShooter, rightShooter,
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


        shooter = new ShootingAction(
                leftShooter,
                rightShooter,
                intake,
                turret,
                hoodServo,
                leftLatch,
                controller
        );

        robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake, leftLatch, hoodServo, light);

        autoAim = new AutoAim(turret, telemetry, true);
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
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        // Ensure PID threads stop when OpMode ends
        try {
            controller.stopVelocityPIDF();
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

