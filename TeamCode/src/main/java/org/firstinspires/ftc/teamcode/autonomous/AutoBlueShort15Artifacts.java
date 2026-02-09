package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.idle;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
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
public class AutoBlueShort15Artifacts extends OpMode {

    public Follower follower;
    public ShootingAction shooter;
    public Turret turretControl;
    public RobotActions robot;
    public ShooterController controller;
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
    int shootDuration = 400; // Duration of the shooting action in milliseconds, this is used in the ShootingAction.shoot() method and can be tuned for faster or slower shooting
    int rampUpDuration = 300; // Duration of the ramp up of the shooter motors in milliseconds, this is used in the ShootingAction.shoot() method and can be tuned for faster or slower ramp up
    int tolerance = 100; // Tolerance in shooting velocity
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
    public GoBildaPinpointDriver odo;

    public PathChain Shoot1;
    public PathChain Shoot2;
    public PathChain OpenGate1;
    public PathChain Take2ndRow;
    public PathChain IntakeGate1;
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

                                new Pose(54.000, 78.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Take2ndRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 78.000),
                                new Pose(42.000, 58.000),
                                new Pose(16.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 60.000),

                                new Pose(60.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        OpenGate1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 72.000),

                                new Pose(16.000, 68.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        IntakeGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.000, 68.000),
                                new Pose(20.000, 57.000),
                                new Pose(12.000, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 50.000),

                                new Pose(54.000, 78.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Take1stRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.000, 78.000),
                                new Pose(46.000, 85.000),
                                new Pose(20.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 84.000),

                                new Pose(60.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Take3rdRow = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 72.000),
                                new Pose(50.000, 32.000),
                                new Pose(16.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Shoot5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 36.000),

                                new Pose(60.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 72.000),

                                new Pose(19.000, 64.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                /* Shoot 1st burst */
                controller.setVelocityPID(idle);
                follower.followPath(Shoot1, true);
                turretControl.spinToHeadingLoop(135, turretSpeedAuto);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                    shooter.shoot(shooterVelocity, shootDuration, rampUpDuration, hoodAngle, tolerance, false);
                    setPathState(2);
                }
                break;
            case 2:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* Take 2nd row */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(Take2ndRow, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* Shoot 2nd burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                    shooter.shoot(shooterVelocity, shootDuration, rampUpDuration, hoodAngle, tolerance, false);
                    setPathState(5);
                }
                break;
            case 5:
                /* Open Gate */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(OpenGate1, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* Take from gate */
                if (!follower.isBusy()) {
                    follower.followPath(IntakeGate1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    while (actionTimer.getElapsedTime() < 500) {
                        intake.setPower(1);
                    }
                    setPathState(8);
                }
                break;
            case 8:
                /* Shoot 3rd burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                    shooter.shoot(shooterVelocity, shootDuration, rampUpDuration, hoodAngle, tolerance, false);
                    setPathState(10);
                }
                break;
            case 10:
                /* Take 1st row */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(Take1stRow, true);
                    setPathState(11);
                }
                break;
            case 11:
                /* Shoot 4th burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                    shooter.shoot(shooterVelocity, shootDuration, rampUpDuration, hoodAngle, tolerance, false);
                    setPathState(13);
                }
                break;
            case 13:
                /* Grab from gate */
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(OpenGate1, true);
                    setPathState(14);
                }
                break;
            case 14:
                /* Collect From Gate */
                if (!follower.isBusy()) {
                    follower.followPath(IntakeGate1, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    while (actionTimer.getElapsedTime() < 500) {
                        intake.setPower(1);
                    }
                    setPathState(16);
                }
                break;
            case 16:
                /* Shoot 5th Burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    turretControl.spinToHeadingLoop(turretAngle, turretSpeedAuto);
                    shooter.shoot(shooterVelocity, shootDuration, rampUpDuration, hoodAngle, tolerance, false);
                    setPathState(18);
                }
                break;
            case 18:
                /* Park */
                if (!follower.isBusy()) {
                    follower.followPath(Park, true);
                    setPathState(19);
                }
                break;
            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    /* Ideally wait until and of auto so odometry can track collisions */
                    setPathState(-1);
                }
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
        // These loop the movements of the robot, these must be called continuously in order to work
        currentPose = new Pose2D(DistanceUnit.INCH, (follower.getPose().getX() - 72), (follower.getPose().getY() - 72), AngleUnit.RADIANS, follower.getPose().getHeading());
        distanceFromGoal = robot.getDistanceFromGoal(currentPose, false);
        shooterVelocity = robot.getShooterRPM(distanceFromGoal);
        hoodAngle = robot.getShooterAngle(distanceFromGoal);
        turretAngle = autoAim.calculateTargetAngle(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.DEGREES));

        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        targetIsRed = false;

        pathTimer = new Timer();
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

        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretControl = new Turret(turret, telemetry);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        light = hardwareMap.get(Servo.class, "light");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        shooter = new ShootingAction(
                leftShooter,
                rightShooter,
                intake,
                turret,
                hoodServo,
                leftLatch,
                rightLatch,
                controller
        );

        robot = new RobotActions(frontLeft, frontRight, backLeft, backRight,
                rightShooter, leftShooter, turret, intake,
                rightLatch, leftLatch, hoodServo, light);

        autoAim = new AutoAim(turret, telemetry, false);
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
        // Ensure PID thread stops when OpMode ends
        controller.stopVelocityPID();
        turretControl.stopVelocityPID();
        // Save final position even on manual termination
        Pose endPose = follower.getPose();
        org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition(
                (endPose.getX() - 72),
                (endPose.getY() - 72),
                Math.toDegrees(endPose.getHeading()),
                turretControl.getCurrentHeading()
        );
    }
}