package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.intermediatePressingLever;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionLong;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.pressLever;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionLong;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
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
public class AutoBlueLong9Artifacts extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double drivePowerMag = 3.0; // the bigger the slower

        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(-17, -64.5), Math.toRadians(0)); //Was (-17, -64.5)

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
        //auto.AutoLong9Artifacts(blue, drive, shooterMotor, intake, transfer, shooter, beginPose);
        //Create starting pose
        //Long Autonomous
        //Shoot first 3 artifacts
        shooterMotor.setVelocity(sniper);
        //Creating autonomous path
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-12, -60))
                .turnTo(Math.toRadians(25))
                .build();

        //Follow the path
        Actions.runBlocking(new SequentialAction(moveToShoot_1));

        shooter.shoot(sniper, shootDurationMs, 0, false);

        //Move to collect first 3 artifacts
        Action collectArtifacts_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-27, -41), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_1));

        Action collectArtifacts_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(-49, -41))
                .build();

        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(new SequentialAction(collectArtifacts_2));
        //intake.setPower(0);
        transfer.setPower(0);
        //Move to shoot second 3 artifacts
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-24, -60), Math.toRadians(22))
                .build();
        intake.setPower(0.8);
        Actions.runBlocking(new SequentialAction(moveToShoot_2));
        shooter.shoot(sniper, shootDurationMs, 500, false);
        //Move to collect last 3 artifacts
        Action collectArtifacts_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-27, -18), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_3));
        //Collect last 3 artifacts
        Action collectArtifacts_4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(-49, -18))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(new SequentialAction(collectArtifacts_4));
        //intake.setPower(0);
        transfer.setPower(0);
        //Move to shoot last 3 artifacts
        shooterMotor.setVelocity(sniper);
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-24, -60), Math.toRadians(21))
                .build();
        intake.setPower(0.8);
        Actions.runBlocking(new SequentialAction(moveToShoot_3));
        shooter.shoot(sniper, shootDurationMs, 0, false);
        //Park the robot
        Action parkRobot = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-40, 0), Math.toRadians(90))
                .build();
        intake.setPower(0.8);
        Actions.runBlocking(new SequentialAction(parkRobot));
    }
}
