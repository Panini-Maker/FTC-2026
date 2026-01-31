package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.artifactHeadingBlue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.artifactHeadingRed;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.collectHumanArtifact1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.collectHumanArtifact2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.collectHumanArtifactIdle;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts1;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.idle;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.intermediateStoppingPoint;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.mirrorHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.mirrorXCoordinate;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionLong;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.pressLever;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.red;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionLong;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniperAuto;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretSpeedAuto;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Autonomous {
    public Vector2d mirrorCoordinates(Vector2d position, String color) {
        int xMultiplier = 1;

        if(color.toLowerCase().equals(blue)) {
            xMultiplier = mirrorXCoordinate;
        }

        return new Vector2d(xMultiplier * position.x, position.y);
    }

    public double findCompensationAngle(double angleToGoal, double currentRobotHeading, double desiredRobotHeading) {
        double error = desiredRobotHeading - currentRobotHeading;

        while (error > 180 || error < -180) {
            if (error > 180) {
                error -= 360;
            } else {
                error += 360;
            }
        }

        if (error > 10) {
            error = 0;
        } else if (error < -10) {
            error = -0;
        }

        return angleToGoal + error;
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

        shooter.shoot(sniper, shootDurationMs, 0);

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
        shooter.shoot(sniper, shootDurationMs, 0);

        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(mirrorCoordinates(parkPositionLong, color))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));
    }

    public void AutoLong9Artifacts(String color,
                                   MecanumDrive drive,
                                   DcMotorEx leftShooter,
                                   DcMotorEx rightShooter,
                                   DcMotor intake,
                                   ShootingAction shooter,
                                   Turret turretControl,
                                   ShooterController shooterController,
                                   Telemetry telemetry,
                                   Pose2d beginPose) throws InterruptedException {
        int headingMultiplier = 1;
        int artifactOrientation = artifactHeadingRed;
        int targetTagID = redTagID;
        int humanArtifactAngle = artifactOrientation - 5;
        int parkingAngle = 90;
        double headingError = 0.0;

        if (color.equalsIgnoreCase(blue)) {
            headingMultiplier = mirrorHeading;
            artifactOrientation = artifactHeadingBlue;
            humanArtifactAngle = artifactOrientation + 5;
            targetTagID = blueTagID;
        }
        //Create starting pose
        //Long Autonomous
        //Shoot first 3 artifacts
        try {
            shooterController.setVelocityPID(sniperAuto);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
        //Creating autonomous path
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionLong, color), Math.toRadians(artifactOrientation))
                .build();

        //Follow the path
        Actions.runBlocking(new SequentialAction(moveToShoot_1));

        double angleToTarget;

        if (color.equalsIgnoreCase(red)) {
            angleToTarget = -115;
        } else {
            angleToTarget = 115;
        }

        // Point turret at target
        try {
            turretControl.spinToHeadingLoop(findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation), turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }

        telemetry.addData("Compensation", findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation));
        telemetry.update();

        try {
            shooter.shoot(sniperAuto, shootDurationMs, 3500);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }

        Action collectArtifacts_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(collectHumanArtifact1, color), Math.toRadians(artifactOrientation))
                .strafeToLinearHeading(mirrorCoordinates(collectHumanArtifactIdle, color), Math.toRadians(artifactOrientation))
                .strafeToLinearHeading(mirrorCoordinates(collectHumanArtifact2, color), Math.toRadians(humanArtifactAngle))
                .build();
        try {
            intake.setPower(1.0);
        } catch (Exception e) {
            telemetry.addData("Intake Error", e.getMessage());
        }
        Actions.runBlocking(new SequentialAction(collectArtifacts_1));

        //Also hit the lever
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionLong, color), Math.toRadians(artifactOrientation))
                .build();
        //shooterController.setVelocityPID(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_2));

        try {
            turretControl.spinToHeadingLoop(findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation), turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }
        telemetry.addData("Compensation", findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation));
        telemetry.update();

        try {
            shooter.shoot(sniperAuto, shootDurationMs, 3500);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }

        Action collectArtifacts_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts1, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_2));

        Action collectArtifacts_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(endCollectingArtifacts1, color), Math.toRadians(artifactOrientation))
                .build();

        try {
            intake.setPower(1);
        } catch (Exception e) {
            telemetry.addData("Intake Error", e.getMessage());
        }
        Actions.runBlocking(new SequentialAction(collectArtifacts_3));


        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionLong, color), Math.toRadians(artifactOrientation))
                .build();

        //shooterController.setVelocityPID(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_3));

        try {
            turretControl.spinToHeadingLoop(findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation), turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }
        telemetry.addData("Compensation", findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation));
        telemetry.update();

        try {
            shooter.shoot(sniperAuto, shootDurationMs, 3500);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }

        try {
            turretControl.spinToHeadingLoop(0, turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }

        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(parkPositionLong, color), Math.toRadians(parkingAngle))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));

        // Save end position for TeleOp
        Pose2d endPose = drive.localizer.getPose();
        try {
            TuningVars.saveEndPosition(endPose.position.x, endPose.position.y, Math.toDegrees(endPose.heading.toDouble()), turretControl.getCurrentHeading());
        } catch (Exception e) {
            // If turret disconnected, save with 0 turret heading
            TuningVars.saveEndPosition(endPose.position.x, endPose.position.y, Math.toDegrees(endPose.heading.toDouble()), 0);
            telemetry.addData("Turret Error", "Could not get turret heading, defaulting to 0");
        }
    }

    public void AutoShort9Artifacts(String color,
                                    MecanumDrive drive,
                                    DcMotorEx leftShooter,
                                    DcMotorEx rightShooter,
                                    DcMotor intake,
                                    ShootingAction shooter,
                                    Turret turretControl,
                                    ShooterController shooterController,
                                    Telemetry telemetry,
                                    Pose2d beginPose) throws InterruptedException {
        int headingMultiplier = 1;
        int artifactOrientation = artifactHeadingRed;
        int targetTagID = redTagID;

        if (color.equalsIgnoreCase(blue)) {
            headingMultiplier = mirrorHeading;
            artifactOrientation = artifactHeadingBlue;
            targetTagID = blueTagID;
        }

        //Short Autonomous
        //Shoot first 3 artifacts

        try {
            shooterController.setVelocityPID(shotgun);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(mirrorCoordinates(shootingPositionShort, color))
                .build();
        Actions.runBlocking(moveToShoot_1);

        double angleToTarget;

        if (color.equalsIgnoreCase(red)) {
            angleToTarget = -135;
        } else {
            angleToTarget = 135;
        }

        // Point turret at target
        //turretControl.spinToHeadingLoop(angleToTarget, turretSpeedAuto);

        try {
            turretControl.spinToHeadingLoop(findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation), turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }
        telemetry.addData("Compensation", findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation));
        telemetry.update();

        // Auto aim using camera (only for first shot)
        //currentHeading = autoAim(tagProcessor, turretControl, currentHeading, targetTagID, 0.3, 3);

        try {
            shooter.shoot(shotgun, shootDurationMs, 2000);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
        //Move to collect more artifacts
        Action moveToCollect_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts3, color), Math.toRadians(artifactOrientation))
                .strafeToLinearHeading(mirrorCoordinates(endCollectingArtifacts3, color), Math.toRadians(artifactOrientation))
                .build();
        try {
            intake.setPower(1);
        } catch (Exception e) {
            telemetry.addData("Intake Error", e.getMessage());
        }
        Actions.runBlocking(moveToCollect_1);
        //intake.setPower(0);
        //Move to shoot again
        //shooterController.setVelocityPID(shotgun);
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(mirrorCoordinates(shootingPositionShort, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(moveToShoot_2);

        try {
            turretControl.spinToHeadingLoop(findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation), turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }
        telemetry.addData("Compensation", findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation));
        telemetry.update();

        try {
            shooter.shoot(shotgun, shootDurationMs, 2000);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }
        //Move to collect more artifacts
        try {
            intake.setPower(1);
        } catch (Exception e) {
            telemetry.addData("Intake Error", e.getMessage());
        }
        Action moveToCollect_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts2, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(moveToCollect_2);

        Action moveToCollect_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(endCollectingArtifacts2, color), Math.toRadians(artifactOrientation))
                .build();
        try {
            intake.setPower(1);
        } catch (Exception e) {
            telemetry.addData("Intake Error", e.getMessage());
        }
        Actions.runBlocking(moveToCollect_3);
        //intake.setPower(0);
        //Move to shoot last time
        //shooterController.setVelocityPID(shotgun);
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(intermediateStoppingPoint, color), Math.toRadians(artifactOrientation))
                .strafeToLinearHeading(mirrorCoordinates(shootingPositionShort, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(moveToShoot_3);

        try {
            turretControl.spinToHeadingLoop(findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation), turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }
        telemetry.addData("Compensation", findCompensationAngle(angleToTarget, drive.localizer.getPose().heading.toDouble(), artifactOrientation));
        telemetry.update();

        try {
            shooter.shoot(shotgun, shootDurationMs, 2000);
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
        }

        try {
            turretControl.spinToHeadingLoop(0, turretSpeedAuto);
        } catch (Exception e) {
            telemetry.addData("Turret Error", e.getMessage());
        }

        //Also hit the lever
        Action hitLever = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(parkPositionShort, color), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(hitLever));

        // Save end position for TeleOp
        Pose2d endPose = drive.localizer.getPose();
        try {
            TuningVars.saveEndPosition(endPose.position.x, endPose.position.y, Math.toDegrees(endPose.heading.toDouble()), turretControl.getCurrentHeading());
        } catch (Exception e) {
            // If turret disconnected, save with 0 turret heading
            TuningVars.saveEndPosition(endPose.position.x, endPose.position.y, Math.toDegrees(endPose.heading.toDouble()), 0);
            telemetry.addData("Turret Error", "Could not get turret heading, defaulting to 0");
        }
    }
}
