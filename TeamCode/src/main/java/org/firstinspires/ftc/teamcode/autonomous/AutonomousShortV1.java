package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootDurationMs;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.rampUpTime;

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
import org.firstinspires.ftc.teamcode.lib.ShootingAction;
import org.firstinspires.ftc.teamcode.lib.SimpleDriveActions;

@Autonomous
public class AutonomousShortV1 extends LinearOpMode {
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

        // Recalibrate Odometry
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ShootingAction shooter = new ShootingAction(leftShooter, transfer, intake);
        SimpleDriveActions drive = new SimpleDriveActions(frontLeft, frontRight, backRight, backLeft, telemetry, odo, leftShooter, intake, transfer, voltageSensor, runTime);

        waitForStart();
        odo.update();
        Pose2D currentPos;
        //Shoot first 3 artifacts
        drive.moveToPosition(0, 50, 0.3, 2, 10000);
        odo.update();
        currentPos = odo.getPosition();
        drive.turnToHeadingWithOdo(-currentPos.getHeading(AngleUnit.DEGREES) - 3.424, 0.15, 1, 2000);
        shooter.shoot(shotgun, shootDurationMs, rampUpTime);

        odo.update();
        currentPos = odo.getPosition();
        //Move to pick up next 3 artifacts
        drive.moveToPosition(0, -4, 0.3, 2, 2000);
        drive.moveToPosition(30 * Math.cos((Math.PI/4) + currentPos.getHeading(AngleUnit.RADIANS)), -30 * Math.sin((Math.PI/4) + currentPos.getHeading(AngleUnit.RADIANS)), 0.3, 2, 8000, true);
        //Shoot next 3 artifacts
        drive.moveToPosition(-30 * Math.cos((Math.PI/4) + currentPos.getHeading(AngleUnit.RADIANS)), 30 * Math.sin((Math.PI/4) + currentPos.getHeading(AngleUnit.RADIANS)), 0.3, 2, 8000);
        drive.moveToPosition(0, 4, 0.3, 2, 2000);
        odo.update();
        currentPos = odo.getPosition();
        drive.turnToHeadingWithOdo(-currentPos.getHeading(AngleUnit.DEGREES) - 3.424, 0.15, 1, 2000);
        shooter.shoot(shotgun, shootDurationMs, rampUpTime);

        //Note: Consider doing 80 - 20 move ratio to decrease time but keep accuracy
    }
}
