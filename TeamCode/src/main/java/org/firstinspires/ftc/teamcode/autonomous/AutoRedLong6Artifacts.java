package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.Red;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AprilTag;
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AutoRedLong6Artifacts extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double drivePowerMag = 3.0; // the bigger the slower
        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(7, -65.5), Math.toRadians(0));

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, drivePowerMag);
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Lower decimation for lighting conditions

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intakeTransfer");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "loading");

        ShootingAction shooter = new ShootingAction(
                shooterMotor,
                transfer,
                intake
        );

        waitForStart();
        org.firstinspires.ftc.teamcode.lib.Autonomous auto = new org.firstinspires.ftc.teamcode.lib.Autonomous();
        auto.AutoLong6Artifacts(Red, drive, shooterMotor, intake, transfer, shooter, beginPose);
    }
}
