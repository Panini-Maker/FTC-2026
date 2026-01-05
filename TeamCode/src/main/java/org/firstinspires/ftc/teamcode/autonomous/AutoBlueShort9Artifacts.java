package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.intermediateStoppingPoint;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.mirrorXCoordinate;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AprilTag;
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * NOTES:
 * 11/17: strafeToLinearHeading() doesn't work
 * 11/18: Back right wheel wheel stopped working, fixed now
 */
@Autonomous
public class AutoBlueShort9Artifacts extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(-40.5, 64.5), Math.toRadians(180));

        //Ineffective
        double drivePowerMag = 6.0; // the bigger the slower

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, drivePowerMag);
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Lower decimation for lighting conditions

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");


        ShootingAction shooter = new ShootingAction(
                leftShooter,
                rightShooter,
                intake,
                turret,
                hoodServo,
                leftLatch,
                rightLatch
        );

        waitForStart();
        org.firstinspires.ftc.teamcode.lib.Autonomous auto = new org.firstinspires.ftc.teamcode.lib.Autonomous();
        auto.AutoShort9Artifacts(blue, drive, leftShooter, rightShooter, intake, shooter, beginPose);
    }
}
