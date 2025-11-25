package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.rampUpTime;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.AprilTag;
import org.firstinspires.ftc.teamcode.lib.CameraMovement;
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.teamcode.lib.SimpleDriveActions;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AutonomousRedV2 extends LinearOpMode {
    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime();

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor transfer = hardwareMap.dcMotor.get("loading");
        DcMotor intake = hardwareMap.dcMotor.get("intakeTransfer");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM); // Set offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Lower decimation for lighting conditions

        // Recalibrate Odometry
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ShootingAction shooter = new ShootingAction(leftShooter, transfer, intake);
        SimpleDriveActions drive = new SimpleDriveActions(frontLeft, frontRight, backRight, backLeft, telemetry, odo, leftShooter, intake, transfer, voltageSensor, runTime);
        CameraMovement camera = new CameraMovement(frontLeft, frontRight, backRight, backLeft, leftShooter, odo, intake, transfer, voltageSensor, telemetry, tagProcessor);

        waitForStart();
        odo.update();
        Pose2D currentPos;
        sleep(1000);

        if(!tagProcessor.getFreshDetections().isEmpty()) {
            //Long Autonomous
            //Shoot first 3 artifacts
            camera.turnToAprilTag(redTagID);
        } else {
            //Short Autonomous
            //Shoot first 3 artifacts
        }
    }
}
