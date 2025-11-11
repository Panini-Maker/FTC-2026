package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TestVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Pose cameraPose = new Pose(0, 0, 0, 0, 0, 0);  X, Y, Z, Pitch, Roll, Yaw

        AprilTagLibrary.Builder aprilTagLibraryBuilder;
        AprilTagLibrary aprilTagLibrary;

        //Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        aprilTagLibraryBuilder = new AprilTagLibrary.Builder().setAllowOverwrite(true);
        //Add all the tags from the given AprilTagLibrary to theAprilTagLibrary.Builder.
        //Get the AprilTagLibrary for the current season.
        aprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        //Add a tag,without pose information,to the AprilTagLibrary.Builder.

        for(int i = 1; i < 30; i++) {
            aprilTagLibraryBuilder.addTag(i, "ID " + i, 200, DistanceUnit.MM);
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
                //.setCameraPose(cameraPose)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                //.setAutoStopLiveView(true)
                .build();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                if (tag.ftcPose != null) {
                    telemetry.addData("tag ID", tag.id);
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                } else {
                    telemetry.addData("tag ID", tag.id);
                    telemetry.addLine("no pose :(");
                }
            }

            telemetry.update();
        }
    }
}
