package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd2;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKf2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.lib.ShooterController;

//@Disabled
@TeleOp
public class ShooterRegression extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors and servos
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        ShooterController shooter = new ShooterController(leftShooter, rightShooter,
                shooterKp, shooterKi, shooterKd, shooterKf,
                shooterKp2, shooterKi2, shooterKd2, shooterKf2,
                voltageSensor, telemetry);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo rightLatch = hardwareMap.get(Servo.class, "rightLatch");
        rightLatch.setDirection(Servo.Direction.REVERSE);
        Servo hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);

        Servo light = hardwareMap.get(Servo.class, "light");

        float shooterSpeed = 1500;

        int latchState = 0;

        double hoodState = 0.15;

        // Pulse intake mode - when enabled, intake only runs when shooter is at speed
        boolean pulseIntakeMode = false;
        double shootingTolerance = 50; // RPM tolerance for pulse mode

        // Intake current monitoring
        double intakeCurrent = 0;
        double maxIntakeCurrent = 0;
        double minIntakeCurrent = Double.MAX_VALUE;
        double avgIntakeCurrent = 0;
        int currentSampleCount = 0;
        double currentSum = 0;

        // Intake jam detection threshold (tune this value based on observations)
        double intakeFullThreshold = 3.0; // Amps - adjust based on your data

        // Air time tracking state machine
        // States: 0=IDLE, 1=RAMPING, 2=FEEDING, 3=TIMING, 4=DONE
        int airTimeState = 0;
        ElapsedTime airTimeTimer = new ElapsedTime();
        double lastAirTimeMs = 0;
        double velocityDipThreshold = 100; // RPM dip to detect ball leaving shooter
        double peakVelocityBeforeDip = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            // Toggle pulse intake mode with gamepad1 right bumper
            if (gamepad1.right_bumper && !gamepad1.start) {
                pulseIntakeMode = !pulseIntakeMode;
                sleep(200); // Debounce
            }

            // Reset current stats with gamepad1 a button
            if (gamepad1.a) {
                maxIntakeCurrent = 0;
                minIntakeCurrent = Double.MAX_VALUE;
                currentSum = 0;
                currentSampleCount = 0;
                avgIntakeCurrent = 0;
                sleep(200); // Debounce
            }

            // Air time tracking with gamepad1 left bumper
            if (gamepad1.left_bumper && !gamepad1.start) {
                if (airTimeState == 0) {
                    // Start: ramp up shooter
                    airTimeState = 1;
                    peakVelocityBeforeDip = 0;
                    sleep(200); // Debounce
                } else if (airTimeState == 3) {
                    // Stop timer: ball has landed
                    lastAirTimeMs = airTimeTimer.milliseconds();
                    airTimeState = 4;
                    sleep(200); // Debounce
                } else if (airTimeState == 4) {
                    // Reset back to idle
                    airTimeState = 0;
                    sleep(200); // Debounce
                }
            }

            // Monitor intake current when intake is running
            intakeCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);

            // Only track stats when intake is actually running
            if (intakeMotor.getPower() > 0) {
                currentSampleCount++;
                currentSum += intakeCurrent;
                avgIntakeCurrent = currentSum / currentSampleCount;

                if (intakeCurrent > maxIntakeCurrent) {
                    maxIntakeCurrent = intakeCurrent;
                }
                if (intakeCurrent < minIntakeCurrent) {
                    minIntakeCurrent = intakeCurrent;
                }
            }

            // Detect if intake is full/jammed
            boolean intakeFull = intakeCurrent > intakeFullThreshold && intakeMotor.getPower() > 0;

            if (gamepad2.aWasPressed()) {
                shooterSpeed += 10;
            } else if (gamepad2.bWasPressed()) {
                shooterSpeed -= 10;
            } else if (gamepad2.xWasPressed()) {
                shooterSpeed += 50;
            } else if (gamepad2.yWasPressed()) {
                shooterSpeed -= 50;
            }

            // Calculate shooter velocity (use Math.abs for reversed motor)
            double shooterAverageSpeed = (Math.abs(rightShooter.getVelocity()) + Math.abs(leftShooter.getVelocity())) / 2;
            boolean shooterAtSpeed = Math.abs(shooterSpeed - shooterAverageSpeed) < shootingTolerance;

            // Air time state machine
            boolean airTimeActive = (airTimeState >= 1 && airTimeState <= 3);
            if (airTimeState == 1) {
                // RAMPING: Run shooter, wait for it to reach speed
                shooter.runShooter(shooterSpeed);
                latchState = 0;
                intakeMotor.setPower(0);
                if (shooterAtSpeed) {
                    airTimeState = 2; // Move to FEEDING
                    peakVelocityBeforeDip = shooterAverageSpeed;
                }
            } else if (airTimeState == 2) {
                // FEEDING: Shooter at speed, run intake to feed 1 ball, watch for velocity dip
                shooter.runShooter(shooterSpeed);
                latchState = 0;
                intakeMotor.setPower(1);
                // Track peak velocity
                if (shooterAverageSpeed > peakVelocityBeforeDip) {
                    peakVelocityBeforeDip = shooterAverageSpeed;
                }
                // Detect velocity dip (ball has left the shooter)
                if (peakVelocityBeforeDip - shooterAverageSpeed > velocityDipThreshold) {
                    airTimeState = 3; // Move to TIMING
                    airTimeTimer.reset();
                    intakeMotor.setPower(0);
                }
            } else if (airTimeState == 3) {
                // TIMING: Ball is in the air, timer running, waiting for user to stop
                shooter.runShooter(shooterSpeed);
                intakeMotor.setPower(0);
            }
            // States 0 (IDLE) and 4 (DONE) don't control motors

            if (!airTimeActive) {
                if (gamepad2.right_bumper) {
                    // Pulse mode: only run intake when shooter is at speed (but only if shooter is running)
                    if (pulseIntakeMode && gamepad2.right_trigger > 0) {
                        intakeMotor.setPower(shooterAtSpeed ? 1 : 0);
                    } else {
                        intakeMotor.setPower(1);
                    }
                    latchState = 1;
                } else if (gamepad2.left_bumper) {
                    intakeMotor.setPower(0.6);
                    latchState = 1;
                } else {
                    intakeMotor.setPower(0);
                }

                if (gamepad2.right_trigger > 0) {
                    latchState = 0;
                    shooter.runShooter(shooterSpeed);
                } else {
                    shooter.resetPIDF();
                    shooter.stopShooter();
                }
            }

            if (gamepad2.dpadUpWasPressed()) {
                if (hoodState < 1) {
                    hoodState += 0.01;
                }
            } else if (gamepad2.dpadDownWasPressed()) {
                if (hoodState > 0) {
                    hoodState -= 0.01;
                }
            }

            if (shooterAtSpeed) {
                light.setPosition(0.5);
            } else {
                light.setPosition(0.28);
            }

            leftLatch.setPosition(latchState);
            rightLatch.setPosition(latchState);

            hood.setPosition(hoodState);

            telemetry.addData("Shooter Speed Target", shooterSpeed);
            telemetry.addData("Shooter Speed", "%.1f", shooterAverageSpeed);
            telemetry.addData("Hood Position", hoodState);
            telemetry.addData("Battery Voltage", "%.2f V", voltageSensor.getVoltage());
            telemetry.addLine("---");
            telemetry.addData("Pulse Intake Mode", pulseIntakeMode ? "ON" : "OFF");
            telemetry.addData("Shooter At Speed", shooterAtSpeed ? "YES" : "NO");
            telemetry.addData("PIDF Debug", shooter.getDebugInfo());
            telemetry.addLine("--- INTAKE CURRENT MONITORING ---");
            telemetry.addData("Current Draw", "%.3f A", intakeCurrent);
            telemetry.addData("Max Current", "%.3f A", maxIntakeCurrent);
            telemetry.addData("Min Current", "%.3f A", minIntakeCurrent == Double.MAX_VALUE ? 0 : minIntakeCurrent);
            telemetry.addData("Avg Current", "%.3f A", avgIntakeCurrent);
            telemetry.addData("Sample Count", currentSampleCount);
            telemetry.addData("Full Threshold", "%.2f A", intakeFullThreshold);
            telemetry.addData("INTAKE FULL?", intakeFull ? ">>> YES <<<" : "No");
            telemetry.addLine("--- AIR TIME TRACKING ---");
            String[] airTimeStateNames = {"IDLE", "RAMPING", "FEEDING", "TIMING", "DONE"};
            telemetry.addData("Air Time State", airTimeStateNames[airTimeState]);
            if (airTimeState == 3) {
                telemetry.addData("Air Time (running)", "%.0f ms", airTimeTimer.milliseconds());
            } else if (airTimeState == 4) {
                telemetry.addData("Air Time Result", "%.0f ms", lastAirTimeMs);
            }
            if (airTimeState == 2) {
                telemetry.addData("Peak Velocity", "%.0f", peakVelocityBeforeDip);
            }
            telemetry.addLine("---");
            telemetry.addLine("GP1 Right Bumper: Toggle Pulse Mode");
            telemetry.addLine("GP1 Left Bumper: Air Time Test");
            telemetry.addLine("GP1 A: Reset Current Stats");
            telemetry.update();

        }
    }

}
