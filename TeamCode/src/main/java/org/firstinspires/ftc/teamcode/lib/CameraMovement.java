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
                drive.turnToHeadingWithOdo(tag.ftcPose.bearing * 0.7, 0.15, 1, 6000);
                /*
                while(Math.abs(tag.ftcPose.bearing) > tolerance) {
                    drive.drive(0, 0, 0.15, 100);
                }
                 */
                odo.resetPosAndIMU();
                sleep(250);
                return;
            }
        }
    }
}