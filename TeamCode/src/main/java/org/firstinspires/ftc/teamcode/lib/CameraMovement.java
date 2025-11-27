package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class CameraMovement {
    //declares hardware and variables
    public Telemetry telemetry;
    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public DcMotorEx shooterMotor;
    public GoBildaPinpointDriver odo;
    public DcMotor intake;
    public DcMotor transfer;
    public VoltageSensor voltageSensor;
    public double forward;
    public double right;
    public double rotate;
    public long timeouts_ms; //timeout in milliseconds
    public ElapsedTime runTime;

    public AprilTagProcessor tagProcessor;
    public SimpleDriveActions drive;

    public CameraMovement(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, DcMotor backLeftMotor,
                          DcMotorEx shooterMotor, GoBildaPinpointDriver odo, DcMotor intake, DcMotor transfer,
                          VoltageSensor voltageSensor, Telemetry telemetry, AprilTagProcessor tagProcessor) {
        this.tagProcessor = tagProcessor;
        this.frontRightMotor = frontRightMotor;
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.shooterMotor = shooterMotor;
        this.odo = odo;
        this.intake = intake;
        this.transfer = transfer;
        this.voltageSensor = voltageSensor;
        this.telemetry = telemetry;
        this.runTime = new ElapsedTime();

        drive = new SimpleDriveActions(frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor,
                telemetry, odo, shooterMotor, intake, transfer, voltageSensor, runTime);
    }

    private double getVoltageCompensatedPower(double power) {
        double currentVoltage = voltageSensor.getVoltage();
        return power * (idealVoltage / currentVoltage);
    }

    public void turnToAprilTag(int tagID) throws InterruptedException {
        while(tagProcessor.getDetections().isEmpty()) {
            drive.turnToHeadingWithOdo(-60, 0.4, 10, 2000);
        }

        for(AprilTagDetection tag : tagProcessor.getDetections()) {
            if(tag.id == tagID) {
                //Currently using 70% of the angle to avoid overshooting
                //Make a function that does not need odometry and only uses the camera
                drive.turnToHeadingWithOdo(tag.ftcPose.bearing * 0.7, 0.3, 1, 6000);
                /*
                while(Math.abs(tag.ftcPose.bearing) > tolerance) {
                    drive.drive(0, 0, 0.15, 100);
                }
                 */
                return;
            }
        }
    }
    public boolean turnToAprilTagNoOdo(int tagID) throws InterruptedException {

        // Check if any AprilTags are detected
        if (tagProcessor.getDetections().isEmpty()) {
            // If no tags are detected, keep turning slowly
            drive.drive(0, 0, 0.15, 100);
        } else {
            // Process detected tags
            for (AprilTagDetection tag : tagProcessor.getDetections()) {
                if (tag.id == tagID) {
                    // If the tag is found, check its bearing
                    if (Math.abs(tag.ftcPose.bearing) > 2) { // Adjust tolerance as needed
                        // Turn towards the tag
                        //Determines the turning direction based on the sign of the bearing.
                        //Positive bearing means the tag is to the right, so the robot turns right (0.15),
                        // and negative bearing means the tag is to the left, so the robot turns left (-0.15).
                        double turnPower = tag.ftcPose.bearing > 0 ? 0.3 : -0.3;
                        drive.drive(0, 0, turnPower, 100);
                    } else {
                        // Stop turning when perpendicular
                        drive.drive(0, 0, 0, 0);
                        return true;
                    }
                }
            }
        }return false;
    }


}