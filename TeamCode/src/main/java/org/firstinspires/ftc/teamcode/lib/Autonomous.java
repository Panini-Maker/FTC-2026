package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.artifactHeadingBlue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.artifactHeadingRed;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.intermediatePressingLever;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.intermediateStoppingPoint;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.mirrorHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.mirrorXCoordinate;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionLong;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.pressLever;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.red;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionLong;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Autonomous {
    public Vector2d mirrorCoordinates(Vector2d position, String color) {
        int xMultiplier = 1;

        if(color.toLowerCase().equals(blue)) {
            xMultiplier = mirrorXCoordinate;
        }

        return new Vector2d(xMultiplier * position.x, position.y);
    }
    public void AutoLong6Artifacts(String color,
                                   MecanumDrive drive,
                                   DcMotorEx shooterMotor,
                                   DcMotor intake,
                                   DcMotor transfer,
                                   ShootingAction shooter,
                                   Pose2d beginPose) throws InterruptedException {
        int headingMultiplier = 1;
        int artifactOrientation = artifactHeadingRed;

        if(color.equalsIgnoreCase("blue")) {
            headingMultiplier = mirrorHeading;
            artifactOrientation = artifactHeadingBlue;
        }

        //Long Autonomous
        //Shoot first 3 artifacts
        shooterMotor.setVelocity(sniper);
        //Creating autonomous path
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeTo(mirrorCoordinates(shootingPositionLong, color))
                .turnTo(Math.toRadians(headingMultiplier * -25))
                .build();

        //Follow the path
        Actions.runBlocking(new SequentialAction(moveToShoot_1));

        shooter.shoot(sniper, shootDurationMs, 0, false);

        Action collectArtifacts_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts1, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_1));

        Action collectArtifacts_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts1, color))
                .build();

        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(new SequentialAction(collectArtifacts_2));
        intake.setPower(0);
        transfer.setPower(0);

        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(mirrorCoordinates(shootingPositionLong, color))
                .turnTo(Math.toRadians(headingMultiplier * -25))
                .build();

        shooterMotor.setVelocity(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_2));
        shooter.shoot(sniper, shootDurationMs, 0, false);

        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(mirrorCoordinates(parkPositionLong, color))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));
    }

    public void AutoLong9Artifacts(String color,
                                   MecanumDrive drive,
                                   DcMotorEx shooterMotor,
                                   DcMotor intake,
                                   DcMotor transfer,
                                   ShootingAction shooter,
                                   Pose2d beginPose) throws InterruptedException {
        int headingMultiplier = 1;
        int artifactOrientation = artifactHeadingRed;

        if(color.equalsIgnoreCase(blue)) {
            headingMultiplier = mirrorHeading;
            artifactOrientation = artifactHeadingBlue;
        }
        //Create starting pose
        //Long Autonomous
        //Shoot first 3 artifacts
        shooterMotor.setVelocity(sniper);
        //Creating autonomous path
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeTo(mirrorCoordinates(shootingPositionLong, color))
                .turnTo(Math.toRadians(headingMultiplier * -25))
                .build();

        //Follow the path
        Actions.runBlocking(new SequentialAction(moveToShoot_1));

        shooter.shoot(sniper, shootDurationMs, 0, false);

        Action collectArtifacts_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts1, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_1));

        Action collectArtifacts_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts1, color))
                .build();

        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(new SequentialAction(collectArtifacts_2));
        //intake.setPower(0);
        transfer.setPower(0);


        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(mirrorCoordinates(shootingPositionLong, color))
                .turnTo(Math.toRadians(headingMultiplier * -25))
                .build();

        shooterMotor.setVelocity(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_2));
        shooter.shoot(sniper, shootDurationMs, 0, false);

        Action collectArtifacts_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts2, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_3));

        Action collectArtifacts_4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts2, color))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(new SequentialAction(collectArtifacts_4));
        //intake.setPower(0);
        transfer.setPower(0);

        //Also hit the lever
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionLong, color), Math.toRadians(headingMultiplier * -25))
                .build();
        shooterMotor.setVelocity(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_3));

        shooter.shoot(sniper, shootDurationMs, 0, false);

        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(pressLever, color), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));
    }

    public void AutoShort9Artifacts(String color,
                                    MecanumDrive drive,
                                    DcMotorEx shooterMotor,
                                    DcMotor intake,
                                    DcMotor transfer,
                                    ShootingAction shooter,
                                    Pose2d beginPose) throws InterruptedException {
        int headingMultiplier = 1;
        int artifactOrientation = artifactHeadingRed;

        if (color.equalsIgnoreCase(blue)) {
            headingMultiplier = mirrorHeading;
            artifactOrientation = artifactHeadingBlue;
        }

        //Short Autonomous
        //Shoot first 3 artifacts

        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionShort, color), Math.toRadians(headingMultiplier * -48))
                .build();
        Actions.runBlocking(moveToShoot_1);
        shooter.shoot(shotgun, shootDurationMs, 0, false);
        //Move to collect more artifacts
        Action moveToCollect_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts3, color), Math.toRadians(artifactOrientation))
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts3, color))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(moveToCollect_1);
        //intake.setPower(0);
        transfer.setPower(0);
        //Move to shoot again
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionShort, color), Math.toRadians(headingMultiplier * -48))
                .build();
        Actions.runBlocking(moveToShoot_2);
        shooter.shoot(shotgun, shootDurationMs, 0, false);
        //Move to collect more artifacts
        Action moveToCollect_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts2, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(moveToCollect_2);

        Action moveToCollect_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts2, color))
                .build();
        intake.setPower(0.8);
        transfer.setPower(0.3);
        Actions.runBlocking(moveToCollect_3);
        //intake.setPower(0);
        transfer.setPower(0);
        //Move to shoot last time
        shooterMotor.setVelocity(shotgun);
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(intermediateStoppingPoint)
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionShort, color), Math.toRadians(headingMultiplier * -48))
                .build();
        Actions.runBlocking(moveToShoot_3);
        shooter.shoot(shotgun, shootDurationMs, 0, false);

        //Also hit the lever
        Action hitLever = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(parkPositionShort, color), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(hitLever));
    }
}
