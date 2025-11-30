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
        Pose2d beginPose = new Pose2d(new Vector2d(-40.5, 64.5), Math.toRadians(0));

        //Ineffective
        double drivePowerMag = 6.0; // the bigger the slower

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, drivePowerMag);
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Lower decimation for lighting conditions

        DcMotorEx shooterMotor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "leftShooter");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intakeTransfer");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "loading");

        ShootingAction shooter = new ShootingAction(
                shooterMotor,
                transfer,
                intake
        );

        waitForStart();
        //org.firstinspires.ftc.teamcode.lib.Autonomous auto = new org.firstinspires.ftc.teamcode.lib.Autonomous();
        //auto.AutoShort9Artifacts(blue, drive, shooterMotor, intake, transfer, shooter, beginPose);
        //Short Autonomous
        //Shoot first 3 artifacts

        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(46))
                .build();
        Actions.runBlocking(moveToShoot_1);
        shooter.shoot(shotgun, shootDurationMs, 0, false);
        //Move to collect more artifacts
        Action moveToCollect_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-24, 6), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-44, 6))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(moveToCollect_1);
        transfer.setPower(0);
        //Move to shoot again
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(50))
                .build();
        Actions.runBlocking(moveToShoot_2);
        shooter.shoot(shotgun, shootDurationMs, 0, false);
        //Move to collect more artifacts
        Action moveToCollect_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-27, -18), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveToCollect_2);

        Action moveToCollect_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(-49, -18))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(moveToCollect_3);
        transfer.setPower(0);
        //Move to shoot last time
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(-33, -12))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(48))
                .build();
        Actions.runBlocking(moveToShoot_3);
        shooter.shoot(shotgun, shootDurationMs, 0, false);

        //Also hit the lever
        Action hitLever = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-52, -18), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(hitLever));
    }
}
