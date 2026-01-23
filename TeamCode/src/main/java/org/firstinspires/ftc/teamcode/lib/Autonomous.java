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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Autonomous {

    /**
     * Auto aim using AprilTag detection to fine-tune turret alignment.
     * Loops until alignment is within tolerance or timeout is reached.
     * @param tagProcessor The AprilTag processor
     * @param turretControl The turret controller
     * @param currentHeading Current turret heading in degrees
     * @param targetTagID The AprilTag ID to aim at
     * @param turretPower Power for turret movement
     * @param timeoutMs Maximum time in milliseconds to attempt alignment
     * @return Updated turret heading
     */
    public double autoAim(AprilTagProcessor tagProcessor, Turret turretControl,
                          double currentHeading, int targetTagID, double turretPower, long timeoutMs) throws InterruptedException {
        double heading = currentHeading;
        long startTime = System.currentTimeMillis();

        while ((System.currentTimeMillis() - startTime) < timeoutMs) {
            // Wait for fresh detections
            sleep(200);

            if (tagProcessor.getFreshDetections() != null && !tagProcessor.getDetections().isEmpty()) {
                sleep(100); // Allow time for processing
                for (AprilTagDetection tag : tagProcessor.getDetections()) {
                    if (tag.id == targetTagID) {
                        // Check if within 1 degree tolerance - exit loop if aligned
                        if (Math.abs(tag.ftcPose.bearing) <= 1.0) {
                            return heading; // Aligned, break out of loop
                        }

                        // Calculate adjustment
                        double adjustment = heading + tag.ftcPose.bearing;

                        // Clamp to turret limits
                        if (adjustment > turretLimitCCW) {
                            adjustment = turretLimitCCW;
                        } else if (adjustment < turretLimitCW) {
                            adjustment = turretLimitCW;
                        }

                        // Move turret and update heading
                        heading = turretControl.spinToHeadingBlocking(adjustment, turretPower, 1000);
                        break;
                    }
                }
            }
        }

        return heading;
    }

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
                                   AprilTagProcessor tagProcessor,
                                   Pose2d beginPose) throws InterruptedException {
        int headingMultiplier = 1;
        int artifactOrientation = artifactHeadingRed;
        int targetTagID = redTagID;

        if (color.equalsIgnoreCase(blue)) {
            headingMultiplier = mirrorHeading;
            artifactOrientation = artifactHeadingBlue;
            targetTagID = blueTagID;
        }
        //Create starting pose
        //Long Autonomous
        //Shoot first 3 artifacts
        shooterController.setVelocityPID(sniperAuto);
        //Creating autonomous path
        Action moveToShoot_1 = drive.actionBuilder(beginPose)
                .strafeTo(mirrorCoordinates(shootingPositionLong, color))
                .build();

        //Follow the path
        Actions.runBlocking(new SequentialAction(moveToShoot_1));

        double angleToTarget;

        if (color.equalsIgnoreCase(red)) {
            angleToTarget = -115;
        } else {
            angleToTarget = 109;
        }

        // Point turret at target
        turretControl.spinToHeadingLoop(angleToTarget, turretSpeedAuto);


        // Auto aim using camera (only for first shot)
        //currentHeading = autoAim(tagProcessor, turretControl, currentHeading, targetTagID, 0.3, 3000);

        /*
        if (color.equalsIgnoreCase(red)) {
            currentHeading = turretControl.spinToHeadingBlocking(currentHeading - 4, 0.2, 2000); // Compensate for consistent error
        } else {
            currentHeading = turretControl.spinToHeadingBlocking(currentHeading + 4, 0.2, 2000); // Compensate for consistent error
        }

         */

        shooter.shoot(sniperAuto, shootDurationMs, 3500);

        Action collectArtifacts_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(collectHumanArtifact1, color), Math.toRadians(artifactOrientation))
                .strafeToConstantHeading(mirrorCoordinates(collectHumanArtifactIdle, color))
                .strafeToConstantHeading(mirrorCoordinates(collectHumanArtifact2, color))
                .build();
        intake.setPower(1.0);
        Actions.runBlocking(new SequentialAction(collectArtifacts_1));

        //Also hit the lever
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(shootingPositionLong, color))
                .build();
        //shooterController.setVelocityPID(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_2));


        shooter.shoot(sniperAuto, shootDurationMs, 3500);

        Action collectArtifacts_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts1, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(new SequentialAction(collectArtifacts_2));

        Action collectArtifacts_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts1, color))
                .build();

        intake.setPower(1);
        Actions.runBlocking(new SequentialAction(collectArtifacts_3));


        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(mirrorCoordinates(shootingPositionLong, color))
                .build();

        //shooterC ontroller.setVelocityPID(sniper);
        Actions.runBlocking(new SequentialAction(moveToShoot_3));

        shooter.shoot(sniperAuto, shootDurationMs, 3500);

        turretControl.spinToHeadingLoop(0, turretSpeedAuto);

        //Move out of zone
        Action moveOutOfZone = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(parkPositionLong, color), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(moveOutOfZone));
    }

    public void AutoShort9Artifacts(String color,
                                    MecanumDrive drive,
                                    DcMotorEx leftShooter,
                                    DcMotorEx rightShooter,
                                    DcMotor intake,
                                    ShootingAction shooter,
                                    Turret turretControl,
                                    ShooterController shooterController,
                                    AprilTagProcessor tagProcessor,
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

        shooterController.setVelocityPID(shotgun);
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
        turretControl.spinToHeadingLoop(angleToTarget, turretSpeedAuto);

        // Auto aim using camera (only for first shot)
        //currentHeading = autoAim(tagProcessor, turretControl, currentHeading, targetTagID, 0.3, 3);

        shooter.shoot(shotgun, shootDurationMs, 2000);
        //Move to collect more artifacts
        Action moveToCollect_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts3, color), Math.toRadians(artifactOrientation))
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts3, color))
                .build();
        intake.setPower(1);
        Actions.runBlocking(moveToCollect_1);
        //intake.setPower(0);
        //Move to shoot again
        //shooterController.setVelocityPID(shotgun);
        Action moveToShoot_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(shootingPositionShort, color))
                .build();
        Actions.runBlocking(moveToShoot_2);

        shooter.shoot(shotgun, shootDurationMs, 2000);
        //Move to collect more artifacts
        intake.setPower(1);
        Action moveToCollect_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(beginCollectingArtifacts2, color), Math.toRadians(artifactOrientation))
                .build();
        Actions.runBlocking(moveToCollect_2);

        Action moveToCollect_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(endCollectingArtifacts2, color))
                .build();
        intake.setPower(1);
        Actions.runBlocking(moveToCollect_3);
        //intake.setPower(0);
        //Move to shoot last time
        //shooterController.setVelocityPID(shotgun);
        Action moveToShoot_3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(mirrorCoordinates(intermediateStoppingPoint, color))
                .strafeToConstantHeading(mirrorCoordinates(shootingPositionShort, color))
                .build();
        Actions.runBlocking(moveToShoot_3);


        shooter.shoot(shotgun, shootDurationMs, 2000);

        turretControl.spinToHeadingLoop(0, turretSpeedAuto);

        //Also hit the lever
        Action hitLever = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(mirrorCoordinates(parkPositionShort, color), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(hitLever));
    }
}
