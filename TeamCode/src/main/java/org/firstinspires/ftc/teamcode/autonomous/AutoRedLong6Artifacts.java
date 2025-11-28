package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;

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
public class AutoRedLong6Artifacts extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double drivePowerMag = 3.0; // the bigger the slower
        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(7, -65.5), Math.toRadians(0));

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, drivePowerMag);
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Lower decimation for lighting conditions

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intakeTransfer");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "loading");

        ShootingAction shooter = new ShootingAction(
                shooterMotor,
                transfer,
                intake
        );

        waitForStart();
        //Create starting pose
        //Long Autonomous
        //Shoot first 3 artifacts
        shooterMotor.setVelocity(sniper);
        //Creating autonomous path
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(12, -60))
                .turn(Math.toRadians(-23))
                .build();

        //Follow the path
        Actions.runBlocking(new SequentialAction(moveToShoot_1));

        shooter.shoot(sniper, shootDurationMs, 0, false);

        Action collectArtifacts_1 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(33, -36), Math.toRadians(0))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_1));

        Action collectArtifacts_2 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(63, -36), Math.toRadians(0))
                .build();

        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(new SequentialAction(collectArtifacts_2));
        intake.setPower(0);
        transfer.setPower(0);

        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(12, -60))
                .turn(Math.toRadians(-20))
                .build();

        shooterMotor.setVelocity(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_2));
        shooter.shoot(sniper, shootDurationMs, 0, false);

        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(12, -36))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));
    }
}
