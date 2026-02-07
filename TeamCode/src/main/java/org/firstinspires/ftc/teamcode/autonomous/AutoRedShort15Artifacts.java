package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous

public class AutoRedShort15Artifacts extends OpMode {

    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(32, 135, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scoreClosePose = new Pose(56, 76, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scoreFarPose = new Pose(61.5, 69.5, Math.toRadians(180));

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1, true);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Take2ndRow, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Shoot2, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(OpenGate1, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(IntakeGate1, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Shoot3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Take1stRow, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* Shoot 4th burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    /* Grab 3rd Row */
                    follower.followPath(Take3rdRow, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* Shoot 5th Burst */
                if (!follower.isBusy()) {
                    follower.followPath(Shoot5);
                    setPathState(10);
                }
                break;
            case 10:
                /* Park */
                if (!follower.isBusy()) {
                    follower.followPath(Park);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
        follower.update();
        autonomousPathUpdate();
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
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
    }


}
