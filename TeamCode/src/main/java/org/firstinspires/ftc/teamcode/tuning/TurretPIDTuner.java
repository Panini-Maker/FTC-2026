package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretMaxPower;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretMinPower;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTicksPerDegree;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTolerance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.Turret;

/**
 * TurretPIDTuner - Used to tune turret PID constants.
 *
 * Controls:
 * - A: Move turret to 90 degrees (safe target position)
 * - B: Move turret to 0 degrees (home position)
 * - X: Move turret to -90 degrees (opposite safe position)
 * - Y: Stop turret and reset PID
 *
 * Telemetry displays:
 * - Current turret angle
 * - Target angle
 * - Error
 * - Current power output
 * - PID constants (Kp, Ki, Kd)
 *
 * To tune:
 * 1. Set Ki and Kd to 0 in TuningVars
 * 2. Increase Kp until turret oscillates around target
 * 3. Increase Kd until oscillation stops
 * 4. If there's steady-state error, increase Ki slightly
 */
@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class TurretPIDTuner extends LinearOpMode {

    // Target positions (safe values that won't damage wires)
    private static final double TARGET_90 = 90.0;   // Safe CCW position
    private static final double TARGET_0 = 0.0;     // Home position
    private static final double TARGET_NEG_90 = -90.0; // Safe CW position

    // PID variables
    private double integral = 0;
    private double previousError = 0;
    private long previousTime = 0;

    // Current target
    private double targetAngle = 0;
    private boolean pidActive = false;
    private Turret turretController;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize turret motor
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        previousTime = System.currentTimeMillis();

        telemetry.addLine("=== Turret PID Tuner ===");
        telemetry.addLine("A: Go to 90°");
        telemetry.addLine("B: Go to 0° (home)");
        telemetry.addLine("X: Go to -90°");
        telemetry.addLine("Y: Stop turret");
        telemetry.addLine("");
        telemetry.addData("Status", "Ready");
        telemetry.update();

        turretController = new Turret(turret, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Handle button inputs
            if (gamepad1.a) {
                targetAngle = TARGET_90;
                pidActive = true;
                resetPID();
            } else if (gamepad1.b) {
                targetAngle = TARGET_0;
                pidActive = true;
                resetPID();
            } else if (gamepad1.x) {
                targetAngle = TARGET_NEG_90;
                pidActive = true;
                resetPID();
            } else if (gamepad1.y) {
                pidActive = false;
                turret.setPower(0);
                resetPID();
            }

            // Get current turret angle
            double currentAngle = turret.getCurrentPosition() / turretTicksPerDegree;
            double error = targetAngle - currentAngle;
            double power = 0;

            // Run PID if active
            if (pidActive) {
                turretController.spinToHeadingLoop(targetAngle, 1.0);
            }

            // Display telemetry
            telemetry.addLine("=== Turret PID Tuner ===");
            telemetry.addLine("");
            telemetry.addData("PID Active", pidActive);
            telemetry.addData("Target Angle", "%.2f°", targetAngle);
            telemetry.addData("Current Angle", "%.2f°", currentAngle);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Motor Power", "%.3f", power);
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
            telemetry.addLine("Controls: A=90°, B=0°, X=-90°, Y=Stop");
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

