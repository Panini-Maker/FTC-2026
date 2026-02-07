package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.beginCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blue;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.endCollectingArtifacts3;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.intermediateStoppingPoint;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.mirrorXCoordinate;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.parkPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingPositionShort;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.targetIsRed;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AprilTag;
import org.firstinspires.ftc.teamcode.lib.ShooterController;
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.teamcode.lib.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * NOTES:
 * 11/17: strafeToLinearHeading() doesn't work
 * 11/18: Back right wheel wheel stopped working, fixed now
 */
@Autonomous
public class AutoBlueShort9Artifacts extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // target color for teleop
        targetIsRed = false;

        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(-38, 64.75), Math.toRadians(180));

        //Ineffective
        double drivePowerMag = 6.0; // the bigger the slower

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, drivePowerMag);

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterController controller = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Turret turretControl = new Turret(turret, telemetry);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        hoodServo.setDirection(Servo.Direction.REVERSE);


        ShootingAction shooter = new ShootingAction(
                leftShooter,
                rightShooter,
                intake,
                turret,
                hoodServo,
                leftLatch,
                rightLatch,
                controller
        );

        waitForStart();
        org.firstinspires.ftc.teamcode.lib.Autonomous auto = new org.firstinspires.ftc.teamcode.lib.Autonomous();

        auto.AutoShort9Artifacts(blue, drive, leftShooter, rightShooter, intake, shooter, turretControl, controller, telemetry, beginPose);
        // Ensure PID thread stops when OpMode ends
        controller.stopVelocityPID();
        turretControl.stopVelocityPID();
        // Save final position even on manual termination
        Pose2d endPose = drive.localizer.getPose();
        org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition(
                endPose.position.x,
                endPose.position.y,
                Math.toDegrees(endPose.heading.toDouble()),
                turretControl.getCurrentHeading()
        );
        /*
        try {
            auto.AutoShort9Artifacts(blue, drive, leftShooter, rightShooter, intake, shooter, turretControl, controller, telemetry, beginPose);
        } finally {
            // Ensure PID thread stops when OpMode ends
            controller.stopVelocityPID();
            turretControl.stopVelocityPID();
            // Save final position even on manual termination
            Pose2d endPose = drive.localizer.getPose();
            org.firstinspires.ftc.teamcode.lib.TuningVars.saveEndPosition(
                endPose.position.x,
                endPose.position.y,
                Math.toDegrees(endPose.heading.toDouble()),
                turretControl.getCurrentHeading()
            );
        }

         */
    }
}
