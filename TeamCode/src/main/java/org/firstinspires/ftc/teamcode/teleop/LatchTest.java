package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple test OpMode to debug left latch servo speed.
 *
 * Controls:
 * - A button: Open latch (position 1)
 * - B button: Close latch (position 0)
 * - X button: Toggle latch
 * - DPad Up/Down: Fine adjust position by 0.05
 */
//@Disabled
@TeleOp(name = "Latch Test", group = "Test")
public class LatchTest extends LinearOpMode {

    private Servo leftLatch;
    private Servo rightLatch;
    private double currentPosition = 0.5; // Start at middle position
    private boolean lastAPressed = false;
    private boolean lastBPressed = false;
    private boolean lastXPressed = false;
    private boolean lastDPadUpPressed = false;
    private boolean lastDPadDownPressed = false;
    private boolean lastRightBumperPressed = false;
    private boolean lastLeftBumperPressed = false;
    private boolean isOpen = false;
    private boolean testingLeftLatch = true; // true = left, false = right
    private boolean testingBothLatches = false; // true = control both at once

    private ElapsedTime toggleTimer = new ElapsedTime();
    private double lastToggleTime = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        rightLatch = hardwareMap.get(Servo.class, "rightLatch");

        telemetry.addLine("Latch Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = Open (pos 1)");
        telemetry.addLine("  B = Close (pos 0)");
        telemetry.addLine("  X = Toggle");
        telemetry.addLine("  DPad Up/Down = Fine adjust");
        telemetry.update();

        waitForStart();

        toggleTimer.reset();

        while (opModeIsActive()) {
            // Left bumper - Toggle both latches mode
            if (gamepad1.left_bumper && !lastLeftBumperPressed) {
                testingBothLatches = !testingBothLatches;
                // Reset position when switching modes
                currentPosition = 0.5;
                leftLatch.setPosition(currentPosition);
                rightLatch.setPosition(currentPosition);
                isOpen = false;
            }
            lastLeftBumperPressed = gamepad1.left_bumper;

            // Right bumper - Switch servo (only when not in both mode)
            if (gamepad1.right_bumper && !lastRightBumperPressed && !testingBothLatches) {
                testingLeftLatch = !testingLeftLatch;
                // Reset position to current servo's needs
                currentPosition = 0.5;
                if (testingLeftLatch) {
                    leftLatch.setPosition(currentPosition);
                } else {
                    rightLatch.setPosition(currentPosition);
                }
                isOpen = false;
            }
            lastRightBumperPressed = gamepad1.right_bumper;

            // A button - Open latch (position 1)
            if (gamepad1.a && !lastAPressed) {
                currentPosition = 1;
                if (testingBothLatches) {
                    leftLatch.setPosition(currentPosition);
                    rightLatch.setPosition(currentPosition);
                } else if (testingLeftLatch) {
                    leftLatch.setPosition(currentPosition);
                } else {
                    rightLatch.setPosition(currentPosition);
                }
                isOpen = true;
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastAPressed = gamepad1.a;

            // B button - Close latch (position 0)
            if (gamepad1.b && !lastBPressed) {
                currentPosition = 0;
                if (testingBothLatches) {
                    leftLatch.setPosition(currentPosition);
                    rightLatch.setPosition(currentPosition);
                } else if (testingLeftLatch) {
                    leftLatch.setPosition(currentPosition);
                } else {
                    rightLatch.setPosition(currentPosition);
                }
                isOpen = false;
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastBPressed = gamepad1.b;

            // X button - Toggle latch
            if (gamepad1.x && !lastXPressed) {
                isOpen = !isOpen;
                currentPosition = isOpen ? 1 : 0;
                if (testingBothLatches) {
                    leftLatch.setPosition(currentPosition);
                    rightLatch.setPosition(currentPosition);
                } else if (testingLeftLatch) {
                    leftLatch.setPosition(currentPosition);
                } else {
                    rightLatch.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastXPressed = gamepad1.x;

            // DPad Up - Increase position
            if (gamepad1.dpad_up && !lastDPadUpPressed) {
                currentPosition = Math.min(1.0, currentPosition + 0.05);
                if (testingBothLatches) {
                    leftLatch.setPosition(currentPosition);
                    rightLatch.setPosition(currentPosition);
                } else if (testingLeftLatch) {
                    leftLatch.setPosition(currentPosition);
                } else {
                    rightLatch.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastDPadUpPressed = gamepad1.dpad_up;

            // DPad Down - Decrease position
            if (gamepad1.dpad_down && !lastDPadDownPressed) {
                currentPosition = Math.max(0.0, currentPosition - 0.05);
                if (testingBothLatches) {
                    leftLatch.setPosition(currentPosition);
                    rightLatch.setPosition(currentPosition);
                } else if (testingLeftLatch) {
                    leftLatch.setPosition(currentPosition);
                } else {
                    rightLatch.setPosition(currentPosition);
                }
                lastToggleTime = toggleTimer.milliseconds();
            }
            lastDPadDownPressed = gamepad1.dpad_down;

            // Calculate time since last toggle
            double timeSinceToggle = toggleTimer.milliseconds() - lastToggleTime;

            // Telemetry
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("       LATCH TEST");
            telemetry.addLine("═══════════════════════════════");
            String activeServoStr;
            if (testingBothLatches) {
                activeServoStr = "BOTH LATCHES";
            } else {
                activeServoStr = testingLeftLatch ? "LEFT LATCH" : "RIGHT LATCH";
            }
            telemetry.addData("Active Servo", activeServoStr);
            telemetry.addData("Current Position", "%.2f", currentPosition);
            telemetry.addData("State", isOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Time Since Last Command", "%.0f ms", timeSinceToggle);
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("  Left Bumper = Toggle both mode");
            telemetry.addLine("  Right Bumper = Switch servo");
            telemetry.addLine("  A = Open (pos 1)");
            telemetry.addLine("  B = Close (pos 0)");
            telemetry.addLine("  X = Toggle");
            telemetry.addLine("  DPad Up/Down = Fine adjust ±0.05");
            telemetry.update();
        }
    }
}

