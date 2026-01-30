package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTolerance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
 * Controls:
 * Gamepad 1 (Driver):
 * - Left stick: Drive (forward/backward, strafe)
 * - Right stick: Rotate
 * - Back: Reset odometry to origin
 *
 * Gamepad 2 (Operator):
 * - A: Calculate and aim at RED goal (press again to recalculate)
 * - B: Calculate and aim at BLUE goal (press again to recalculate)
 * - X: Manual turret CCW (stops auto aim)
 * - Y: Manual turret CW (stops auto aim)
 * - Left trigger: Stop turret and cancel auto aim
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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Variables
        double power = 0.6;
        double turretPower = 0.5;
        boolean isRed = true;
        double calculatedTargetAngle = 0;
        boolean targetCalculated = false;
        double currentHeading = 0;

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
            double currentXOdo = odo.getPosX(DistanceUnit.INCH);
            double currentYOdo = odo.getPosY(DistanceUnit.INCH);
            double currentHeadingOdo = odo.getHeading(AngleUnit.DEGREES);

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

            // === AUTO AIM CONTROL ===
            // A = Start/restart auto aim for RED goal (uses thread for continuous updates)
            if (gamepad2.aWasPressed()) {
                // Stop existing auto aim if running
                if (autoAimController.isRunning()) {
                    autoAimController.stopAutoAim();
                }

                isRed = true;
                autoAimController.setTeam(true);
                targetCalculated = true;

                // Start auto aim thread - it will continuously update
                autoAimController.startAutoAim();

                // Get debug values for display
                double[] debugVals = autoAimController.getDebugValues();
                double[] storedPos = autoAimController.getStoredPosition();
                telemetry.addLine(">>> RED AUTO AIM STARTED <<<");
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

            // B = Start/restart auto aim for BLUE goal (uses thread for continuous updates)
            if (gamepad2.bWasPressed()) {
                // Stop existing auto aim if running
                if (autoAimController.isRunning()) {
                    autoAimController.stopAutoAim();
                }

                isRed = false;
                autoAimController.setTeam(false);
                targetCalculated = true;

                // Start auto aim thread - it will continuously update
                autoAimController.startAutoAim();

                // Get debug values for display
                double[] debugVals = autoAimController.getDebugValues();
                double[] storedPos = autoAimController.getStoredPosition();
                telemetry.addLine(">>> BLUE AUTO AIM STARTED <<<");
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
            double currentTurretAngle = autoAimController.getCurrentTurretHeading();

            // Get current target for display
            if (autoAimController.isRunning()) {
                calculatedTargetAngle = autoAimController.getTargetAngle();
            }

            // Left trigger = Stop auto aim thread and turret
            if (gamepad2.left_trigger > 0.1) {
                if (autoAimController.isRunning()) {
                    autoAimController.stopAutoAim();
                }
                turret.setPower(0);
                targetCalculated = false;
            }
            // X = Manual CCW (stops auto aim thread)
            else if (gamepad2.x && currentHeading < turretLimitCCW) {
                if (autoAimController.isRunning()) {
                    autoAimController.stopAutoAim();
                }
                targetCalculated = false;
                currentHeading = turretController.spinToHeading(currentHeading + 30, turretPower);
            }
            // Y = Manual CW (stops auto aim thread)
            else if (gamepad2.y && currentHeading > turretLimitCW) {
                if (autoAimController.isRunning()) {
                    autoAimController.stopAutoAim();
                }
                targetCalculated = false;
                currentHeading = turretController.spinToHeading(currentHeading - 30, turretPower);
            }
            // Auto aim thread handles turret movement when running
            else if (!autoAimController.isRunning() && !targetCalculated) {
                turretController.stopTurret();
            }

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
            telemetry.addData("Thread Running", autoAimController.isRunning());
            telemetry.addData("Target Team", isRed ? "RED (72,72)" : "BLUE (-72,72)");
            if (autoAimController.isRunning() || targetCalculated) {
                telemetry.addData("Target Angle", "%.2f°", calculatedTargetAngle);
                telemetry.addData("Error", "%.2f°", calculatedTargetAngle - currentTurretAngle);
                telemetry.addData("On Target", autoAimController.isOnTarget());
            }
            telemetry.addLine("");
            telemetry.addLine("--- Controls ---");
            telemetry.addLine("A: Start RED | B: Start BLUE");
            telemetry.addLine("(press again to restart)");
            telemetry.addLine("LT: Stop | X/Y: Manual");
            telemetry.addLine("BACK: Reset Odometry");
            telemetry.update();
        }

        // Cleanup - stop auto aim thread when OpMode ends
        autoAimController.stopAutoAim();
    }
}

