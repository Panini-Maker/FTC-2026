package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.ShooterController;

/**
 * ShooterPIDFTuner - Used to tune shooter PIDF constants.
 *
 * Controls:
 * - A: Run shooter at Shotgun speed (TeleOp)
 * - B: Run shooter at Sniper speed
 * - X: Run shooter at Shotgun speed (Auto)
 * - Y: Stop shooter
 * - DPad Up: Increase target velocity by 50
 * - DPad Down: Decrease target velocity by 50
 * - Right Bumper: Increase target velocity by 10
 * - Left Bumper: Decrease target velocity by 10
 *
 * Telemetry displays:
 * - Target velocity
 * - Current velocity (left, right, average)
 * - Error
 * - PIDF constants (Kp, Ki, Kd, Kf)
 *
 * To tune:
 * 1. Set Ki, Kd, and Kf to 0 in TuningVars
 * 2. Increase Kp until shooter oscillates around target velocity
 * 3. Increase Kd until oscillation stops
 * 4. If there's steady-state error, increase Ki slightly
 * 5. Add Kf for feedforward (scales with target velocity)
 */
@Config
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterPIDTuner extends LinearOpMode {

    // Target velocity
    private double targetVelocity = 1500;
    private double velocityAdjustment = 0;
    private boolean shooterActive = false;

    // Raw power mode for Kf tuning
    private double rawPower = 0.5;
    private boolean rawPowerMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize shooter motors
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        ElapsedTime runTime = new ElapsedTime();

        // Initialize shooter controller with PIDF
        ShooterController shooter = new ShooterController(leftShooter, rightShooter,
                shooterKp, shooterKi, shooterKd, shooterKf, telemetry);

        telemetry.addLine("=== Shooter PIDF Tuner ===");
        telemetry.addLine("A: Start PIDF control");
        telemetry.addLine("B: Start raw power mode (for Kf tuning)");
        telemetry.addLine("X: Stop motors");
        telemetry.addLine("Y: Stop shooter");
        telemetry.addLine("DPad Up/Down: ±50 RPM (velocity)");
        telemetry.addLine("DPad Left/Right: ±0.05 (raw power)");
        telemetry.addLine("Bumpers: ±10 RPM");
        telemetry.addLine("");
        telemetry.addData("Status", "Ready");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("targetVelocity", targetVelocity);
            packet.put("currentVelocity", (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0);
            dashboard.sendTelemetryPacket(packet);

            // Y button to stop shooter
            if (gamepad1.y) {
                shooterActive = false;
                rawPowerMode = false;
                shooter.stopShooter();
            } else if (gamepad1.a) {
                // A button: Start PIDF control mode
                targetVelocity += velocityAdjustment;
                velocityAdjustment = 0;
                shooterActive = true;
                rawPowerMode = false;
            } else if (gamepad1.b) {
                // B button: Start raw power mode (for Kf tuning)
                rawPowerMode = true;
                shooterActive = false;
            } else if (gamepad1.x) {
                // X button: Stop motors
                rawPowerMode = false;
                shooterActive = false;
                leftShooter.setPower(0.0);
                rightShooter.setPower(0.0);
            }

            // Handle velocity adjustment buttons (with debounce)
            if (gamepad1.dpad_up) {
                velocityAdjustment += 50;
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            } else if (gamepad1.dpad_down) {
                velocityAdjustment -= 50;
                if (velocityAdjustment < -targetVelocity) velocityAdjustment = -targetVelocity;
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            }

            // Handle raw power adjustment (dpad left/right)
            if (gamepad1.dpad_right) {
                rawPower += 0.05;
                if (rawPower > 1.0) rawPower = 1.0;
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            } else if (gamepad1.dpad_left) {
                rawPower -= 0.05;
                if (rawPower < 0.0) rawPower = 0.0;
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            }

            if (gamepad1.right_bumper) {
                intake.setPower(0.9);
            } else {
                intake.setPower(0.0);
            }

            // Get current velocities
            double leftVelocity = leftShooter.getVelocity();
            double rightVelocity = rightShooter.getVelocity();
            double avgVelocity = (leftVelocity + rightVelocity) / 2.0;
            double error = targetVelocity - avgVelocity;

            // Run shooter based on mode
            if (rawPowerMode) {
                // Raw power mode for Kf tuning
                leftShooter.setPower(rawPower);
                rightShooter.setPower(rawPower);
            } else if (shooterActive) {
                shooter.runShooter(targetVelocity);
            }

            // Calculate suggested Kf based on raw power and velocity (for Kf tuning help)
            double suggestedKf = (avgVelocity > 100) ? rawPower / avgVelocity : 0;

            // Display telemetry
            telemetry.addLine("=== Shooter PIDF Tuner ===");
            telemetry.addLine("");
            telemetry.addData("Mode", rawPowerMode ? "RAW POWER (Kf Tuning)" : (shooterActive ? "PIDF Control" : "STOPPED"));
            telemetry.addLine("");

            if (rawPowerMode) {
                telemetry.addLine("--- Raw Power Mode (for Kf tuning) ---");
                telemetry.addData("Raw Power", "%.2f", rawPower);
                telemetry.addData("Resulting Velocity", "%.0f ticks/sec", avgVelocity);
                telemetry.addData("Suggested Kf", "%.6f (power/velocity)", suggestedKf);
                telemetry.addLine("");
                telemetry.addLine("Adjust power with DPad Left/Right (±0.05)");
            } else {
                telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
                telemetry.addData("Pending Adjustment", "%.0f ticks/sec", velocityAdjustment);
                telemetry.addData("New Target (press A)", "%.0f ticks/sec", (targetVelocity + velocityAdjustment));
            }

            telemetry.addLine("");
            telemetry.addLine("--- Current Velocities ---");
            telemetry.addData("Left Shooter", "%.0f ticks/sec", leftVelocity);
            telemetry.addData("Right Shooter", "%.0f ticks/sec", rightVelocity);
            telemetry.addData("Average", "%.0f ticks/sec", avgVelocity);
            if (!rawPowerMode) {
                telemetry.addData("Error", "%.0f ticks/sec", error);
            }
            telemetry.addLine("");
            telemetry.addLine("--- PIDF Constants (edit in TuningVars) ---");
            telemetry.addData("Kp", shooterKp);
            telemetry.addData("Ki", shooterKi);
            telemetry.addData("Kd", shooterKd);
            telemetry.addData("Kf", shooterKf);
            telemetry.addLine("");
            telemetry.addLine("A=PIDF Mode, B=Raw Power, X=Stop, Y=Off");
            telemetry.addLine("DPad Up/Down=±50 vel, Left/Right=±0.05 power");
            telemetry.update();

            // Small delay to prevent busy waiting
            sleep(10);
        }

        // Stop shooter when OpMode ends
        shooter.stopShooter();
    }
}

