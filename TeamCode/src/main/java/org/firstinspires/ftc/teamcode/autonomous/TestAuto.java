package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.red;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

@Autonomous
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double drivePowerMag = 3.0; // the bigger the slower

        //Create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(7, -64.75), Math.toRadians(0));

        //Create RR drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, drivePowerMag);
        AprilTagProcessor tagProcessor = AprilTag.defineCameraFunctions(hardwareMap);
        tagProcessor.setDecimation(0.5f); // Lower decimation for lighting conditions

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

        Turret turretControl = new Turret(turret);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");


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

        Action testCollectHumanArtifacts = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(60, -60))
                .build();

        intake.setPower(1.0);
        Actions.runBlocking(testCollectHumanArtifacts);

        Action moveBackToRePosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(56,-60))
                .strafeToConstantHeading(new Vector2d(60,-64.75))
                .build();

        Actions.runBlocking(moveBackToRePosition);
        Thread.sleep(1500);
        intake.setPower(0.0);
    }
}
