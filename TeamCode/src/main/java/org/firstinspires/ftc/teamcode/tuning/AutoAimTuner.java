package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.Turret;

/**
 * AutoAimTuner - Used to test and tune Auto Aim functionality.
 *
 * Only has drivetrain and turret - no shooter or intake.
 * Auto aim calculates and executes when a button is pressed.
 * It does NOT update until that button is pressed again.
 *
 * Controls (all on Gamepad 1):
 * - Left stick: Drive (forward/backward, strafe)
 * - Right stick: Rotate
 * - A: Calculate and aim at RED goal (press again to recalculate)
 * - B: Calculate and aim at BLUE goal (press again to recalculate)
 * - X: Manual turret CCW (stops auto aim)
 * - Y: Manual turret CW (stops auto aim)
 * - Left trigger: Stop turret and cancel auto aim
 * - Back: Reset odometry to origin
 *
 * This allows you to:
 * 1. Drive the robot to a position
 * 2. Press A or B to calculate AND start aiming at the goal
 * 3. The turret will move to and hold that angle
 * 4. Drive the robot around - turret stays at same angle
 * 5. Press A or B again to recalculate based on new position
 */
@TeleOp(name = "Auto Aim Tuner", group = "Tuning")
public class AutoAimTuner extends LinearOpMode {

    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drivetrain
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize turret
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Turret turretController = new Turret(turret, telemetry);

        // Initialize AutoAim (default to red)
        AutoAim autoAimController = new AutoAim(turret, telemetry, true);

        // Configure odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Variables
        double power = 0.8;
        double turretPower = 1.0;
        boolean isRed = true;
        double calculatedTargetAngle = 0;
        boolean targetCalculated = false;
        double currentHeading = 0;

        odo.resetPosAndIMU();
        sleep(250);
        telemetry.addLine("Odometry RESET!");
        telemetry.update();

        // Don't reset odometry automatically - let driver choose
        telemetry.addLine("=== Auto Aim Tuner ===");
        telemetry.addLine("Press BACK on gamepad1 to reset odometry");
        telemetry.addLine("");
        telemetry.addData("Current Heading", "%.2f", odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        // Allow reset during init
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.back) {
                odo.resetPosAndIMU();
                sleep(250);
                telemetry.addLine("Odometry RESET!");
                telemetry.update();
                sleep(500);
            }
            odo.update();
            telemetry.addLine("=== Auto Aim Tuner ===");
            telemetry.addLine("Press BACK to reset odometry");
            telemetry.addData("Odo Heading", "%.2f", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Odo X", "%.2f", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Odo Y", "%.2f", odo.getPosY(DistanceUnit.INCH));
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentXOdo = pos.getX(DistanceUnit.INCH);
            double currentYOdo = pos.getY(DistanceUnit.INCH);
            double currentHeadingOdo = pos.getHeading(AngleUnit.DEGREES);

            // Reset odometry during runtime
            if (gamepad1.back) {
                odo.resetPosAndIMU();
                sleep(250);
            }

            // Update robot position in auto aim controller
            autoAimController.updateRobotPosition(currentXOdo, currentYOdo, currentHeadingOdo,
                    autoAimController.getCurrentTurretHeading());

            // === DRIVETRAIN ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = (y + x + rx) / Math.max(Math.abs(y + x + rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
            double frontRightPower = (y - x - rx) / Math.max(Math.abs(y - x - rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
            double backLeftPower = (y - x + rx) / Math.max(Math.abs(y - x + rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
            double backRightPower = (y + x - rx) / Math.max(Math.abs(y + x - rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            if (gamepad1.start) {
                odo.resetPosAndIMU();
                sleep(250);
            }

            // === AUTO AIM CONTROL ===
            // A = Calculate target angle for RED goal and use Turret.spinToHeading
            if (gamepad1.a) {
                isRed = true;
                autoAimController.setTeam(true);
                targetCalculated = true;

                // Get debug values for display
                double[] debugVals = autoAimController.getDebugValues();
                double[] storedPos = autoAimController.getStoredPosition();
                telemetry.addLine(">>> RED AUTO AIM CALCULATED <<<");
                telemetry.addLine("-- Odo Values (this loop) --");
                telemetry.addData("Odo X", "%.2f", currentXOdo);
                telemetry.addData("Odo Y", "%.2f", currentYOdo);
                telemetry.addData("Odo Heading", "%.2f", currentHeadingOdo);
                telemetry.addLine("-- Stored in AutoAim --");
                telemetry.addData("Stored X", "%.2f", storedPos[0]);
                telemetry.addData("Stored Y", "%.2f", storedPos[1]);
                telemetry.addData("Stored Heading", "%.2f", storedPos[2]);
                telemetry.addLine("-- Calculation --");
                telemetry.addData("Abs Angle", "%.2f", debugVals[0]);
                telemetry.addData("Adj Heading", "%.2f", debugVals[1]);
                telemetry.addData("Relative", "%.2f", debugVals[2]);
                telemetry.addData("Target", "%.2f", debugVals[3]);
            }

            // B = Calculate target angle for BLUE goal and use Turret.spinToHeading
            if (gamepad1.b) {
                isRed = false;
                autoAimController.setTeam(false);
                targetCalculated = true;

                // Get debug values for display
                double[] debugVals = autoAimController.getDebugValues();
                double[] storedPos = autoAimController.getStoredPosition();
                telemetry.addLine(">>> BLUE AUTO AIM CALCULATED <<<");
                telemetry.addLine("-- Odo Values (this loop) --");
                telemetry.addData("Odo X", "%.2f", currentXOdo);
                telemetry.addData("Odo Y", "%.2f", currentYOdo);
                telemetry.addData("Odo Heading", "%.2f", currentHeadingOdo);
                telemetry.addLine("-- Stored in AutoAim --");
                telemetry.addData("Stored X", "%.2f", storedPos[0]);
                telemetry.addData("Stored Y", "%.2f", storedPos[1]);
                telemetry.addData("Stored Heading", "%.2f", storedPos[2]);
                telemetry.addLine("-- Calculation --");
                telemetry.addData("Abs Angle", "%.2f", debugVals[0]);
                telemetry.addData("Adj Heading", "%.2f", debugVals[1]);
                telemetry.addData("Relative", "%.2f", debugVals[2]);
                telemetry.addData("Target", "%.2f", debugVals[3]);
            }

            // === TURRET CONTROL ===
            calculatedTargetAngle = autoAimController.calculateTargetAngle(currentXOdo, currentYOdo, currentHeadingOdo);
            double currentTurretAngle = turretController.getCurrentHeading();

            // Left trigger = Stop turret and cancel auto aim
            if (gamepad1.left_trigger > 0.1) {
                turretController.stopTurret();
                targetCalculated = false;
            }
            // X = Manual CCW
            else if (gamepad1.x && currentHeading < turretLimitCCW) {
                targetCalculated = false;
                currentHeading = turretController.spinToHeading(currentHeading + 30, turretPower);
            }
            // Y = Manual CW
            else if (gamepad1.y && currentHeading > turretLimitCW) {
                targetCalculated = false;
                currentHeading = turretController.spinToHeading(currentHeading - 30, turretPower);
            }
            // Use Turret.spinToHeading to move to calculated target angle
            else if (targetCalculated) {
                //turretController.spinToHeading(calculatedTargetAngle, turretPower);
            }
            else {
                turretController.stopTurret();
            }

            turretController.spinToHeadingLoop(calculatedTargetAngle, turretPower);

            // Update current heading for manual control
            currentHeading = currentTurretAngle;

            // === TELEMETRY ===
            telemetry.addLine("=== AUTO AIM TUNER ===");
            telemetry.addLine("");
            telemetry.addLine("--- Robot Position ---");
            telemetry.addData("X", "%.2f in", currentXOdo);
            telemetry.addData("Y", "%.2f in", currentYOdo);
            telemetry.addData("Heading", "%.2f°", currentHeadingOdo);
            telemetry.addLine("");
            telemetry.addLine("--- Turret ---");
            telemetry.addData("Current Angle", "%.2f°", currentTurretAngle);
            telemetry.addData("Turret Power", "%.2f", turret.getPower());
            telemetry.addLine("");
            telemetry.addLine("--- Auto Aim ---");
            telemetry.addData("Target Calculated", targetCalculated);
            telemetry.addData("Target Team", isRed ? "RED (72,72)" : "BLUE (-72,72)");
            if (targetCalculated) {
                telemetry.addData("Target Angle", "%.2f°", calculatedTargetAngle);
                telemetry.addData("Error", "%.2f°", calculatedTargetAngle - currentTurretAngle);
                telemetry.addData("On Target", turretController.isWithinTolerance(calculatedTargetAngle));
            }
            telemetry.addLine("");
            telemetry.addLine("--- Controls ---");
            telemetry.addLine("A: RED target | B: BLUE target");
            telemetry.addLine("(press again to recalculate)");
            telemetry.addLine("LT: Stop | X/Y: Manual");
            telemetry.addLine("BACK: Reset Odometry");
            telemetry.update();
        }

        // Cleanup when OpMode ends
        turretController.stopTurret();
        turretController.stopVelocityPID();
    }
}

