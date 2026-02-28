package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.cameraResolutionHeight;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.cameraResolutionWidth;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class AprilTag {
    private static final String CAMERA_NAME = "Webcam 1";
    private static VisionPortal visionPortal;

    // Original calibration was at 640x480
    private static final double CALIB_WIDTH = 640.0;
    private static final double CALIB_HEIGHT = 480.0;
    private static final double CALIB_FX = 549.651;
    private static final double CALIB_FY = 549.651;
    private static final double CALIB_CX = 317.108;
    private static final double CALIB_CY = 236.644;

    public static AprilTagProcessor defineCameraFunctions(HardwareMap hardwareMap) {
        // Close existing portal if it exists
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        // Scale lens intrinsics to current resolution
        double scaleX = cameraResolutionWidth / CALIB_WIDTH;
        double scaleY = cameraResolutionHeight / CALIB_HEIGHT;
        double fx = CALIB_FX * scaleX;
        double fy = CALIB_FY * scaleY;
        double cx = CALIB_CX * scaleX;
        double cy = CALIB_CY * scaleY;

        //Pose cameraPose = new Pose(0, 0, 0, 0, 0, 0);  X, Y, Z, Pitch, Roll, Yaw

        AprilTagLibrary.Builder aprilTagLibraryBuilder;
        AprilTagLibrary aprilTagLibrary;

        //Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        aprilTagLibraryBuilder = new AprilTagLibrary.Builder().setAllowOverwrite(true);
        //Add all the tags from the given AprilTagLibrary to theAprilTagLibrary.Builder.
        //Get the AprilTagLibrary for the current season.
        aprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        //Add a tag,without pose information,to the AprilTagLibrary.Builder.

        for(int i = 20; i < 25; i++) {
            aprilTagLibraryBuilder.addTag(i, "ID " + i, 6.5, DistanceUnit.INCH);
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
                .setLensIntrinsics(fx, fy, cx, cy)
                //.setCameraPose(cameraPose)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(cameraResolutionWidth, cameraResolutionHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();

        return tagProcessor;
    }

    public static VisionPortal getVisionPortal() {
        return visionPortal;
    }

    /**
     * Set manual exposure to limit effective FPS.
     * exposureMs = 33 → ~30fps, exposureMs = 50 → ~20fps, etc.
     * Must be called AFTER the camera is streaming (not while stopped).
     * @param exposureMs exposure time in milliseconds
     * @param gain       camera gain (higher = brighter but noisier)
     * @return true if exposure was set successfully
     */
    public static boolean setManualExposure(int exposureMs, int gain) {
        if (visionPortal == null) return false;

        // Wait for camera to be streaming before setting controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        // Set manual exposure mode
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        if (exposureControl != null) {
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
        }

        // Set gain
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(gain);
        }

        return true;
    }

    public static void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
