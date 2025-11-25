package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.cameraResolutionHeight;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.cameraResolutionWidth;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {
    private static final String CAMERA_NAME = "Webcam 1";
    public static AprilTagProcessor defineCameraFunctions(HardwareMap hardwareMap) {
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
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(cameraResolutionWidth, cameraResolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                //.setAutoStopLiveView(true)
                .build();

        return tagProcessor;
    }
}
