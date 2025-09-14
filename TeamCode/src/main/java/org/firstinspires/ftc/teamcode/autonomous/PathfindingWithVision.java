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

//Not finished
//@Autonomous
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

        for (int i = 1; i < 22; i++) {
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

        while (tagProcessor.getDetections().isEmpty() && opModeIsActive()) {
            drive.CalcSpeeds(0, 0, 1, 0.3,
                    frontLeft, frontRight, backLeft, backRight);
            telemetry(tagProcessor);
            telemetry.addLine("Searching for a tag...");
            telemetry.update();
        }

        drive.CalcSpeeds(0, 0, 0, 0,
                frontLeft, frontRight, backLeft, backRight);

        //Lines 85 and 89 need to be changed as there is an array index
        //out of bounds exception
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        double distance = getDistances(tag.ftcPose.x, tag.ftcPose.y,
                tag.ftcPose.yaw, tag.ftcPose.bearing);

        while (tag.ftcPose.x != distance) {
            tag = tagProcessor.getDetections().get(0);
            if (distance >= 0) {
                drive.CalcSpeeds(0, 1, 0, 0.3,
                        frontLeft, frontRight, backLeft, backRight);
            } else {
                drive.CalcSpeeds(0, -1, 0, 0.3,
                        frontLeft, frontRight, backLeft, backRight);
            }
        }


    }

    private double getDistances(double x, double y, double yaw, double bearing) {
        double distance;
        if (x >= 0) {
            distance = y * Math.sin(yaw);
        } else {
            distance = -y * Math.sin(yaw);
        }

        return distance;
    }

    private void telemetry(AprilTagProcessor tagProcessor) {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            if (tag.ftcPose != null) {
                telemetry.addData("tag ID", tag.id);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation", tag.ftcPose.elevation);
            } else {
                telemetry.addData("tag ID", tag.id);
                telemetry.addLine("no pose :(");
            }
        }

        telemetry.update();
    }
}
