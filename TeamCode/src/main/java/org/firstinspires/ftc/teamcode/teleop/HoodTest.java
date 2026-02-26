package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple test OpMode to debug hood servo.
 *
 * Controls:
 * - A button: Set hood to sniper position (0.5)
 * - B button: Set hood to shotgun position (0.42)
 * - X button: Toggle between sniper/shotgun
 * - Y button: Set to middle position (0.5)
 * - DPad Up/Down: Fine adjust position by 0.01
 * - DPad Left/Right: Coarse adjust position by 0.05
 * - Left Bumper: Disable PWM (release servo)
 * - Right Bumper: Enable PWM (hold position)
 */
@Disabled
@TeleOp(name = "Hood Test", group = "Test")
public class HoodTest extends LinearOpMode {

    private ServoImplEx hoodServo;
    private double currentPosition = 0.5; // Start at middle position
    private boolean lastAPressed = false;
    private boolean lastBPressed = false;
    private boolean lastXPressed = false;
    private boolean lastYPressed = false;
    private boolean lastDPadUpPressed = false;
    private boolean lastDPadDownPressed = false;
    private boolean lastDPadLeftPressed = false;
    private boolean lastDPadRightPressed = false;
    private boolean lastLeftBumperPressed = false;
    private boolean lastRightBumperPressed = false;
    private boolean isSniper = true; // true = sniper (0.5), false = shotgun (0.42)
    private boolean pwmEnabled = true;

    private static final double SNIPER_POSITION = 0.5;
    private static final double SHOTGUN_POSITION = 0.42;

    private ElapsedTime toggleTimer = new ElapsedTime();
    private double lastToggleTime = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware as ServoImplEx for PWM control
        hoodServo = hardwareMap.get(ServoImplEx.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Hood Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = Sniper (0.5)");
        telemetry.addLine("  B = Shotgun (0.42)");
        telemetry.addLine("  X = Toggle preset");
        telemetry.addLine("  Y = Middle (0.5)");
        telemetry.addLine("  DPad Up/Down = Fine ±0.01");
        telemetry.addLine("  DPad Left/Right = Coarse ±0.05");
        telemetry.addLine("  LB = Disable PWM | RB = Enable PWM");
        telemetry.update();

        waitForStart();

        toggleTimer.reset();
        hoodServo.setPosition(currentPosition);

        while (opModeIsActive()) {
            // Left bumper - Disable PWM (release servo)
            if (gamepad1.left_bumper && !lastLeftBumperPressed) {
                hoodServo.setPwmDisable();
                pwmEnabled = false;
            }
            lastLeftBumperPressed = gamepad1.left_bumper;

            // Right bumper - Enable PWM (hold position)
            if (gamepad1.right_bumper && !lastRightBumperPressed) {
                hoodServo.setPwmEnable();
                hoodServo.setPosition(currentPosition);
                pwmEnabled = true;
            }
            lastRightBumperPressed = gamepad1.right_bumper;

            // A button - Sniper position (0.5)
            if (gamepad1.a && !lastAPressed) {
                currentPosition = SNIPER_POSITION;
                isSniper = true;
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastAPressed = gamepad1.a;

            // B button - Shotgun position (0.42)
            if (gamepad1.b && !lastBPressed) {
                currentPosition = SHOTGUN_POSITION;
                isSniper = false;
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastBPressed = gamepad1.b;

            // X button - Toggle between sniper/shotgun
            if (gamepad1.x && !lastXPressed) {
                isSniper = !isSniper;
                currentPosition = isSniper ? SNIPER_POSITION : SHOTGUN_POSITION;
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastXPressed = gamepad1.x;

            // Y button - Middle position (0.5)
            if (gamepad1.y && !lastYPressed) {
                currentPosition = 0.5;
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastYPressed = gamepad1.y;

            // DPad Up - Fine increase position (+0.01)
            if (gamepad1.dpad_up && !lastDPadUpPressed) {
                currentPosition = Math.min(1.0, currentPosition + 0.01);
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastDPadUpPressed = gamepad1.dpad_up;

            // DPad Down - Fine decrease position (-0.01)
            if (gamepad1.dpad_down && !lastDPadDownPressed) {
                currentPosition = Math.max(0.0, currentPosition - 0.01);
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastDPadDownPressed = gamepad1.dpad_down;

            // DPad Right - Coarse increase position (+0.05)
            if (gamepad1.dpad_right && !lastDPadRightPressed) {
                currentPosition = Math.min(1.0, currentPosition + 0.05);
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastDPadRightPressed = gamepad1.dpad_right;

            // DPad Left - Coarse decrease position (-0.05)
            if (gamepad1.dpad_left && !lastDPadLeftPressed) {
                currentPosition = Math.max(0.0, currentPosition - 0.05);
                if (pwmEnabled) {
                    hoodServo.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastDPadLeftPressed = gamepad1.dpad_left;

            // Calculate time since last command
            double timeSinceToggle = toggleTimer.milliseconds() - lastToggleTime;

            // Telemetry
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("       HOOD TEST");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("PWM Status", pwmEnabled ? "ENABLED (holding)" : "DISABLED (free)");
            telemetry.addData("Current Position", "%.3f", currentPosition);
            telemetry.addData("Preset", isSniper ? "SNIPER (0.5)" : "SHOTGUN (0.42)");
            telemetry.addData("Time Since Last Command", "%.0f ms", timeSinceToggle);
            telemetry.addLine("");
            telemetry.addLine("Presets:");
            telemetry.addLine("  A = Sniper (0.5)");
            telemetry.addLine("  B = Shotgun (0.42)");
            telemetry.addLine("  X = Toggle | Y = Middle");
            telemetry.addLine("");
            telemetry.addLine("Adjustments:");
            telemetry.addLine("  DPad Up/Down = Fine ±0.01");
            telemetry.addLine("  DPad Left/Right = Coarse ±0.05");
            telemetry.addLine("");
            telemetry.addLine("PWM Control:");
            telemetry.addLine("  LB = Disable (release servo)");
            telemetry.addLine("  RB = Enable (hold position)");
            telemetry.update();
        }
    }
}

