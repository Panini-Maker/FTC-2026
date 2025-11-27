package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
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

@Autonomous
public class AutoRedShort9Artifacts extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(7, -65.5), Math.toRadians(0));

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
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
        //Short Autonomous
        //Shoot first 3 artifacts

        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .build();
        Actions.runBlocking(moveToShoot_1);
        shooter.shoot(shotgun, shootDurationMs, 0, false);

        //Move to collect more artifacts
        Action moveToCollect_1 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(48, 12), Math.toRadians(0))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(moveToCollect_1);
        intake.setPower(0);
        transfer.setPower(0);
        //Move to shoot again
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .build();
        Actions.runBlocking(moveToShoot_2);
        shooter.shoot(shotgun, shootDurationMs, 0, false);
        //Move to collect more artifacts
        Action moveToCollect_2 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(33, -12), Math.toRadians(0))
                .build();
        Actions.runBlocking(moveToCollect_2);

        Action moveToCollect_3 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(60, -12), Math.toRadians(0))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(moveToCollect_3);
        intake.setPower(0);
        transfer.setPower(0);
        //Move to shoot last time
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .build();
        Actions.runBlocking(moveToShoot_3);
        shooter.shoot(shotgun, shootDurationMs, 0, false);
        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(36, 12))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));
    }
}
