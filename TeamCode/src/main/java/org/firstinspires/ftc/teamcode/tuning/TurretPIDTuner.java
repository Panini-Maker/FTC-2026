package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretMaxPower;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretMinPower;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTicksPerDegree;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTolerance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Turret;

/**
 * TurretPIDTuner - Used to tune turret PID constants.
 * Plots error graph on FTC Dashboard for analysis.
 *
 * Controls:
 * - A: Start PID control to target heading
 * - Y: Stop turret and reset PID
 * - DPad Up/Down: Adjust target heading by ±10°
 * - DPad Left/Right: Adjust target heading by ±5°
 * - Bumpers: Fine adjust target heading by ±1°
 * - B: Go to 0° (home position)
 * - X: Go to current position (stop movement)
 *
 * Turret Limits: CCW = +175°, CW = -130°
 *
 * Telemetry displays:
 * - Current turret angle
 * - Target angle
 * - Error
 * - Current power output
 * - PID constants (Kp, Ki, Kd)
 *
 * FTC Dashboard graphs:
 * - Error over time
 * - Current angle vs Target angle
 *
 * To tune:
 * 1. Set Ki and Kd to 0 in TuningVars
 * 2. Increase Kp until turret oscillates around target
 * 3. Increase Kd until oscillation stops
 * 4. If there's steady-state error, increase Ki slightly
 */
@Config
@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class TurretPIDTuner extends LinearOpMode {

    // Target heading (adjustable via DPad)
    private double targetAngle = 0;
    private double headingAdjustment = 0;
    private boolean pidActive = false;

    // PID variables
    private double integral = 0;
    private double previousError = 0;
    private long previousTime = 0;

    // Turret controller
    private Turret turretController;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize turret motor
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        previousTime = System.currentTimeMillis();

        ElapsedTime runTime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("=== Turret PID Tuner ===");
        telemetry.addLine("A: Start PID to target");
        telemetry.addLine("B: Go to 0° (home)");
        telemetry.addLine("X: Stop at current position");
        telemetry.addLine("Y: Stop turret");
        telemetry.addLine("DPad Up/Down: ±10°");
        telemetry.addLine("DPad Left/Right: ±5°");
        telemetry.addLine("Bumpers: ±1°");
        telemetry.addLine("");
        telemetry.addLine("Presets (GP2): A=90°, B=-90°");
        telemetry.addLine("");
        telemetry.addData("Limits", "CW: %.0f° to CCW: %.0f°", turretLimitCW, turretLimitCCW);
        telemetry.addData("Status", "Ready");
        telemetry.update();

        turretController = new Turret(turret, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Get current turret angle
            double currentAngle = turret.getCurrentPosition() / turretTicksPerDegree;
            double effectiveTarget = targetAngle + headingAdjustment;

            // Clamp effective target to turret limits
            effectiveTarget = Math.max(turretLimitCW, Math.min(turretLimitCCW, effectiveTarget));

            double error = effectiveTarget - currentAngle;

            // Send data to FTC Dashboard for graphing
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Error (deg)", error);
            packet.put("Current Angle (deg)", currentAngle);
            packet.put("Target Angle (deg)", effectiveTarget);
            packet.put("PID Active", pidActive ? 1 : 0);
            dashboard.sendTelemetryPacket(packet);

            // Handle button inputs
            if (gamepad1.a) {
                // A button: Start PID control to target
                targetAngle += headingAdjustment;
                targetAngle = Math.max(turretLimitCW, Math.min(turretLimitCCW, targetAngle));
                headingAdjustment = 0;
                pidActive = true;
                resetPID();
            } else if (gamepad1.b) {
                // B button: Go to home (0°)
                targetAngle = 0;
                headingAdjustment = 0;
                pidActive = true;
                resetPID();
            } else if (gamepad1.x) {
                // X button: Stop at current position
                targetAngle = currentAngle;
                headingAdjustment = 0;
                pidActive = false;
                turret.setPower(0);
                resetPID();
            } else if (gamepad1.y) {
                // Y button: Stop turret completely
                pidActive = false;
                turret.setPower(0);
                resetPID();
            }

            // Handle presets (Gamepad 2) - only 90° and -90°
            if (gamepad2.a) {
                // Preset: 90° CCW
                targetAngle = 90;
                headingAdjustment = 0;
                pidActive = true;
                resetPID();
            } else if (gamepad2.b) {
                // Preset: -90° CW
                targetAngle = -90;
                headingAdjustment = 0;
                pidActive = true;
                resetPID();
            }

            // Handle heading adjustment buttons (with debounce)
            // DPad Up/Down: ±10°
            if (gamepad1.dpad_up) {
                headingAdjustment += 10;
                // Clamp to limits
                if (targetAngle + headingAdjustment > turretLimitCCW) {
                    headingAdjustment = turretLimitCCW - targetAngle;
                }
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            } else if (gamepad1.dpad_down) {
                headingAdjustment -= 10;
                // Clamp to limits
                if (targetAngle + headingAdjustment < turretLimitCW) {
                    headingAdjustment = turretLimitCW - targetAngle;
                }
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            }

            // DPad Left/Right: ±5°
            if (gamepad1.dpad_right) {
                headingAdjustment += 5;
                if (targetAngle + headingAdjustment > turretLimitCCW) {
                    headingAdjustment = turretLimitCCW - targetAngle;
                }
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            } else if (gamepad1.dpad_left) {
                headingAdjustment -= 5;
                if (targetAngle + headingAdjustment < turretLimitCW) {
                    headingAdjustment = turretLimitCW - targetAngle;
                }
                runTime.reset();
                while (runTime.milliseconds() < 200) {}
            }

            // Bumpers: ±1° (fine adjustment)
            if (gamepad1.right_bumper) {
                headingAdjustment += 1;
                if (targetAngle + headingAdjustment > turretLimitCCW) {
                    headingAdjustment = turretLimitCCW - targetAngle;
                }
                runTime.reset();
                while (runTime.milliseconds() < 150) {}
            } else if (gamepad1.left_bumper) {
                headingAdjustment -= 1;
                if (targetAngle + headingAdjustment < turretLimitCW) {
                    headingAdjustment = turretLimitCW - targetAngle;
                }
                runTime.reset();
                while (runTime.milliseconds() < 150) {}
            }

            // Run PID if active (using full power - no limit)
            if (pidActive) {
                turretController.spinToHeadingLoop(effectiveTarget, 1.0);
            }

            // Display telemetry
            telemetry.addLine("=== Turret PID Tuner ===");
            telemetry.addLine("");
            telemetry.addData("PID Active", pidActive);
            telemetry.addLine("");
            telemetry.addLine("--- Heading ---");
            telemetry.addData("Current Angle", "%.2f°", currentAngle);
            telemetry.addData("Target Angle", "%.2f°", targetAngle);
            telemetry.addData("Pending Adjustment", "%.2f°", headingAdjustment);
            telemetry.addData("New Target (press A)", "%.2f°", effectiveTarget);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addLine("");
            telemetry.addLine("--- Limits ---");
            telemetry.addData("CW Limit", "%.0f°", turretLimitCW);
            telemetry.addData("CCW Limit", "%.0f°", turretLimitCCW);
            telemetry.addLine("");
            telemetry.addLine("--- PID Constants (edit in TuningVars) ---");
            telemetry.addData("Kp", turretKp);
            telemetry.addData("Ki", turretKi);
            telemetry.addData("Kd", turretKd);
            telemetry.addLine("");
            telemetry.addLine("--- Other Settings ---");
            telemetry.addData("Min Power", turretMinPower);
            telemetry.addData("Max Power", turretMaxPower);
            telemetry.addData("Tolerance", turretTolerance);
            telemetry.addLine("");
            telemetry.addLine("GP1: A=Go, B=Home, X=Stop Here, Y=Off");
            telemetry.addLine("GP1: DPad U/D=±10°, L/R=±5°, Bumpers=±1°");
            telemetry.addLine("GP2: A=90°, B=-90°");
            telemetry.update();

            // Small delay to prevent busy waiting
            sleep(10);
        }

        // Stop turret when OpMode ends
        turretController.stopTurret();
        turretController.stopVelocityPID();
    }


    /**
     * Calculates PID output to move turret to target angle.
     */
    private double calculatePID(double targetAngle, double currentAngle) {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - previousTime) / 1000.0; // Convert to seconds

        if (dt <= 0) dt = 0.01; // Default if time hasn't changed

        double error = targetAngle - currentAngle;

        // Proportional term
        double P = turretKp * error;

        // Integral term (with anti-windup)
        integral += error * dt;
        integral = Math.max(-100, Math.min(100, integral)); // Clamp integral
        double I = turretKi * integral;

        // Derivative term
        double derivative = (error - previousError) / dt;
        double D = turretKd * derivative;

        // Calculate total output
        double output = P + I + D;

        // Apply minimum power to overcome friction (only if not within tolerance)
        if (Math.abs(error) > turretTolerance) {
            if (Math.abs(output) < turretMinPower) {
                output = Math.copySign(turretMinPower, output);
            }
        } else {
            output = 0; // Within tolerance, stop
        }

        // Clamp to max power
        output = Math.max(-turretMaxPower, Math.min(turretMaxPower, output));

        // Save for next iteration
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    /**
     * Resets the PID controller state.
     */
    private void resetPID() {
        integral = 0;
        previousError = 0;
        previousTime = System.currentTimeMillis();
    }
}

