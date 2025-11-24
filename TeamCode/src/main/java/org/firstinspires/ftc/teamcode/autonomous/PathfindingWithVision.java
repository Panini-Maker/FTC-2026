package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//Not finished
@Autonomous
public class PathfindingWithVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        MecanumDriveAuto drive = new MecanumDriveAuto();

        AprilTagLibrary.Builder aprilTagLibraryBuilder;
        AprilTagProcessor.Builder aprilTagProcessorBuilder;
        AprilTagLibrary aprilTagLibrary;

        //Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        aprilTagLibraryBuilder = new AprilTagLibrary.Builder().setAllowOverwrite(true);
        //Add all the tags from the given AprilTagLibrary to theAprilTagLibrary.Builder.
        //Get the AprilTagLibrary for the current season.
        aprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        //Add a tag,without pose information,to the AprilTagLibrary.Builder.

        for (int i = 1; i < 24; i++) {
            aprilTagLibraryBuilder.addTag(i, "ID " + i, 6.21875, DistanceUnit.INCH);
        }

        //Build the AprilTag library and assign it to a variable.
        aprilTagLibrary = aprilTagLibraryBuilder.build();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(aprilTagLibrary)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();

        // --- Constants to Tune --- //
        final double Y_TARGET = 12.0;      // How far from the tag you want to stop (inches)

        // Proportional gain constants (tune these to adjust robot behavior)
        final double Kp_TURN = 0.02;     // Proportional gain for turning
        final double Kp_STRAFE = 0.03;   // Proportional gain for strafing
        final double Kp_FORWARD = 0.04;  // Proportional gain for driving forward/backward

        final double MAX_SPEED = 0.4; // Maximum robot speed

        // Tolerance constants for the "done" condition
        final double X_TOLERANCE = 0.5; // Side-to-side tolerance (inches)
        final double Y_TOLERANCE = 0.5; // Forward/backward tolerance (inches)
        final double BEARING_TOLERANCE = 1.0; // Rotational tolerance (degrees)

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (detections.isEmpty()) {
                // No tag is visible, so search for one by rotating.
                telemetry.addData("Status", "Searching for a tag...");
                drive.CalcSpeeds(0, 0, 1, 0.3,
                        frontLeft, frontRight, backLeft, backRight);
            } else {
                // A tag is visible, so start alignment.
                AprilTagDetection tag = detections.get(0); // Align to the first tag detected.
                telemetry.addData("Status", "Aligning to Tag ID: " + tag.id);

                // Calculate errors from the target position.
                double bearingError = tag.ftcPose.bearing; // Error in rotation
                double xError = tag.ftcPose.x;           // Error in strafe (left/right)
                double yError = tag.ftcPose.y - Y_TARGET;  // Error in distance (forward/backward)

                // Check if the robot is aligned within the tolerances.
                if (Math.abs(xError) <= X_TOLERANCE && Math.abs(yError) <= Y_TOLERANCE && Math.abs(bearingError) <= BEARING_TOLERANCE) {
                    // Robot is aligned. Stop the motors.
                    telemetry.addLine("Aligned!");
                    drive.CalcSpeeds(0, 0, 0, 0,
                            frontLeft, frontRight, backLeft, backRight);
                } else {
                    // Robot is not aligned. Calculate motor powers using proportional control.
                    double turnPower = bearingError * Kp_TURN;
                    double strafePower = xError * Kp_STRAFE;
                    double forwardPower = yError * Kp_FORWARD;

                    // Combine powers and send to CalcSpeeds. The MecanumDriveAuto class will handle scaling.
                    drive.CalcSpeeds(forwardPower, strafePower, turnPower, MAX_SPEED,
                            frontLeft, frontRight, backLeft, backRight);
                }

                // Add telemetry for debugging.
                telemetry.addData("Bearing Error", bearingError);
                telemetry.addData("X Error", xError);
                telemetry.addData("Y Error", yError);
            }
            telemetry.update();
        }
    }
}
