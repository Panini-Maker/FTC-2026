package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgun;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgunTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniperAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ShooterController;

/**
 * ShooterPIDTuner - Used to tune shooter PID constants.
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
 * - PID constants (Kp, Ki, Kd)
 *
 * To tune:
 * 1. Set Ki and Kd to 0 in TuningVars
 * 2. Increase Kp until shooter oscillates around target velocity
 * 3. Increase Kd until oscillation stops
 * 4. If there's steady-state error, increase Ki slightly
 */
@Config
@TeleOp(name = "Shooter PID Tuner", group = "Tuning")
public class ShooterPIDTuner extends LinearOpMode {

    // Target velocity
    private double targetVelocity = 0;
    private boolean shooterActive = false;

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

        // Initialize shooter controller with PID
        ShooterController shooter = new ShooterController(leftShooter, rightShooter,
                shooterKp, shooterKi, shooterKd, telemetry);

        telemetry.addLine("=== Shooter PID Tuner ===");
        telemetry.addLine("A: Shotgun TeleOp speed");
        telemetry.addLine("B: Sniper speed");
        telemetry.addLine("X: Shotgun Auto speed");
        telemetry.addLine("Y: Stop shooter");
        telemetry.addLine("DPad Up/Down: ±50 RPM");
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

            // Handle preset velocity buttons
            if (gamepad1.a) {
                targetVelocity = shotgunTeleOp;
                shooterActive = true;
                shooter.resetPID();
            } else if (gamepad1.b) {
                targetVelocity = sniper;
                shooterActive = true;
                shooter.resetPID();
            } else if (gamepad1.x) {
                targetVelocity = shotgun;
                shooterActive = true;
                shooter.resetPID();
            } else if (gamepad1.y) {
                shooterActive = false;
                shooter.stopShooter();
            }

            // Handle velocity adjustment buttons (with debounce)
            if (gamepad1.dpad_up) {
                targetVelocity += 50;
                sleep(200); // Debounce
            } else if (gamepad1.dpad_down) {
                targetVelocity -= 50;
                if (targetVelocity < 0) targetVelocity = 0;
                sleep(200); // Debounce
            } else if (gamepad1.right_bumper) {
                targetVelocity += 10;
                sleep(150); // Debounce
            } else if (gamepad1.left_bumper) {
                targetVelocity -= 10;
                if (targetVelocity < 0) targetVelocity = 0;
                sleep(150); // Debounce
            }

            // Get current velocities
            double leftVelocity = leftShooter.getVelocity();
            double rightVelocity = rightShooter.getVelocity();
            double avgVelocity = (leftVelocity + rightVelocity) / 2.0;
            double error = targetVelocity - avgVelocity;

            // Run shooter if active
            if (shooterActive) {
                shooter.runShooter(targetVelocity);
            }

            // Display telemetry
            telemetry.addLine("=== Shooter PID Tuner ===");
            telemetry.addLine("");
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
            telemetry.addLine("");
            telemetry.addLine("--- Current Velocities ---");
            telemetry.addData("Left Shooter", "%.0f RPM", leftVelocity);
            telemetry.addData("Right Shooter", "%.0f RPM", rightVelocity);
            telemetry.addData("Average", "%.0f RPM", avgVelocity);
            telemetry.addData("Error", "%.0f RPM", error);
            telemetry.addLine("");
            telemetry.addLine("--- PID Constants (edit in TuningVars) ---");
            telemetry.addData("Kp", shooterKp);
            telemetry.addData("Ki", shooterKi);
            telemetry.addData("Kd", shooterKd);
            telemetry.addLine("");
            telemetry.addLine("--- Preset Speeds ---");
            telemetry.addData("Shotgun TeleOp (A)", shotgunTeleOp);
            telemetry.addData("Sniper (B)", sniper);
            telemetry.addData("Shotgun Auto (X)", shotgun);
            telemetry.addLine("");
            telemetry.addLine("Controls: A/B/X=Presets, Y=Stop");
            telemetry.addLine("DPad=±50, Bumpers=±10");
            telemetry.update();

            // Small delay to prevent busy waiting
            sleep(10);
        }

        // Stop shooter when OpMode ends
        shooter.stopShooter();
    }
}

