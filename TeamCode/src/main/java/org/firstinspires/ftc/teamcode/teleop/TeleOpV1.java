package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndTurretHeading;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndX;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.autoEndY;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoXOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.odoYOffset;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redTagID;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingToleranceTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shotgunTeleOp;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretLimitCW;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTolerance;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.AutoAim;
import org.firstinspires.ftc.teamcode.lib.ShooterController;
import org.firstinspires.ftc.teamcode.lib.Turret;


@Disabled
@TeleOp
public class TeleOpV1 extends LinearOpMode {

    GoBildaPinpointDriver odo;

    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Turret turretController = new Turret(turret, telemetry);

        // Initialize AutoAim (default to red team, can be toggled)
        AutoAim autoAimController = new AutoAim(turret, telemetry, true);

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        Servo leftLatch = hardwareMap.get(Servo.class, "leftLatch");
        Servo light = hardwareMap.get(Servo.class, "light");

        hoodServo.setDirection(Servo.Direction.REVERSE);

        // Configure odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(odoXOffset, odoYOffset, DistanceUnit.MM); // Set offsets
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        int latchState = 0;

        double power = 0.8; //Used teo limit speed for testing/safety

        double turretPower = 1.0;
        boolean shooterMode = true;
        boolean targetIsRed = true;
        double shooterPower;
        int target = redTagID;
        boolean isAligned = false;
        double currentHeading = 0.0;
        double distanceToGoal = 0.0;

        //Auto Aim and Auto Shoot variables
        boolean autoAim = false;
        boolean toggleAutoAim = true;
        boolean autoShoot = false;
        boolean toggleAutoShoot = true;
        boolean safeShooting = true;

        //May be useful later
        double currentXOdo;
        double currentYOdo;
        double currentHeadingOdo;

        // Initialize odometry
        // Only reset if we don't have saved position from autonomous
        if (autoEndX == 0 && autoEndY == 0 && autoEndHeading == 0) {
            // No saved position - reset odometry to origin
            odo.resetPosAndIMU();
            // Wait for IMU to calibrate
            odo.resetPosAndIMU();
            sleep(250);
            telemetry.addData("Odometry", "Reset to origin, IMU calibrating...");
        } else {
            // Have saved position from autonomous - set it without full reset
            odo.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH, autoEndX, autoEndY, AngleUnit.DEGREES, autoEndHeading));
            telemetry.addData("Odometry", "Loaded from Auto: X=%.1f, Y=%.1f, H=%.1f", autoEndX, autoEndY, autoEndHeading);
        }

        // Initialize turret heading from autonomous (if available)
        currentHeading = autoEndTurretHeading;

        // Get position
        odo.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            // Get position
            odo.update();
            Pose2D pos = odo.getPosition();

            //odo.setOffsets();
            currentXOdo = pos.getX(DistanceUnit.INCH);
            currentYOdo = pos.getY(DistanceUnit.INCH);
            currentHeadingOdo = pos.getHeading(AngleUnit.DEGREES);

            if (gamepad2.yWasPressed()) {
                shooterMode = !shooterMode;
            }

            if (gamepad2.aWasPressed()) {
                targetIsRed = !targetIsRed;
                autoAimController.setTeam(targetIsRed); // Update auto aim target
            }

            if (targetIsRed) {
                target = redTagID;
                telemetry.addData("Target Color:", "Red");
            } else {
                target = blueTagID;
                telemetry.addData("Target Color:", "Blue");
            }

            if (shooterMode) {
                shooterPower = shotgunTeleOp;
                telemetry.addData("Shooter Mode:", "Shotgun");
                //shooter.setPIDConstants(0.0025,0.00099,0);
                hoodServo.setPosition(0.4);
            } else {
                shooterPower = sniper;
                telemetry.addData("Shooter Mode", "Sniper");
                //shooter.setPIDConstants(0.0025,0.00099,0);
                hoodServo.setPosition(0.6);
            }

            //Auto Aim no longer toggled by drivers

            if (gamepad1.yWasPressed()) {
                safeShooting = !safeShooting;
            }

            // Reset alignment status when robot moves or auto-aim is toggled off
            if (!autoAim) {
                isAligned = false;
            }

            // Intake controls with error handling
            try {
                if (gamepad2.right_bumper) {
                    if (!(leftLatch.getPosition() == 1) && (leftShooter.getPower() == 0) && (rightShooter.getPower() == 0)) {
                        latchState = 0;
                    }
                    intake.setPower(1);
                } else if (gamepad2.left_bumper) {
                    intake.setPower(-0.8);
                } else {
                    intake.setPower(0);
                }
            } catch (Exception e) {
                telemetry.addData("Intake Error", e.getMessage());
            }

            // Shooter controls with error handling
            try {
                if ((gamepad2.right_trigger > 0) || (isAligned && autoShoot)) {
                    if (!(leftLatch.getPosition() == 0)) {
                        latchState = 1;
                    }
                    shooter.runShooter(shooterPower);
                } else if (gamepad2.left_trigger > 0) {
                    shooter.resetPID();
                    leftShooter.setPower(-0.3);
                    rightShooter.setPower(-0.3);
                    //for intaking from human players
                } else {
                    shooter.resetPID();
                    shooter.stopShooter();
                }
            } catch (Exception e) {
                telemetry.addData("Shooter Error", e.getMessage());
            }

            if (gamepad1.left_trigger > 0) {
                power = 0.6;
            } else if (gamepad1.right_trigger > 0) {
                power = 1;
            } else {
                power = 0.8;
            }

            // Drivetrain controls with error handling
            try {
                double y = -gamepad1.left_stick_y; // Forward/Backward
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x; // Turning

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]

                double frontLeftPower = (y + x + rx) / Math.max(Math.abs(y + x + rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
                double frontRightPower = (y - x - rx) / Math.max(Math.abs(y - x - rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
                double backLeftPower = (y - x + rx) / Math.max(Math.abs(y - x + rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());
                double backRightPower = (y + x - rx) / Math.max(Math.abs(y + x - rx), 1) * power * (idealVoltage / voltageSensor.getVoltage());

                frontLeft.setPower(frontLeftPower);
                frontRight.setPower(frontRightPower);
                backLeft.setPower(backLeftPower);
                backRight.setPower(backRightPower);
            } catch (Exception e) {
                telemetry.addData("Drivetrain Error", e.getMessage());
            }

            // Intake and Transfer Controls
            // right trigger to shooting balls
            //left trigger for intaking and transferring balls
            // if both triggers are pressed, the robot will do both actions simultaneously
            // right button is the outtake in case we intake too many artifacts

            // Turret controls with error handling
            // Auto Aim activates when shooter is ramping up (right trigger pressed)
            // Manual control (X/B buttons) overrides auto aim
            try {
                boolean shooterRampingUp = (gamepad2.right_trigger > 0);
                boolean manualTurretControl = gamepad2.x || gamepad2.b;

                // Update robot position for auto aim (thread uses these values)
                autoAimController.updateRobotPosition(currentXOdo, currentYOdo, currentHeadingOdo,
                    autoAimController.getCurrentTurretHeading());

                if (manualTurretControl) {
                    // Manual control takes priority - stop auto aim thread if running
                    if (autoAimController.isRunning()) {
                        autoAimController.stopAutoAim();
                    }

                    if (gamepad2.x && (currentHeading <= turretLimitCCW)) {
                        currentHeading = turretController.spinToHeading(currentHeading + 30, turretPower);
                    } else if (gamepad2.b && (currentHeading >= turretLimitCW)) {
                        currentHeading = turretController.spinToHeading(currentHeading - 30, turretPower);
                    }
                    autoAim = false;
                    isAligned = false;
                } else if (shooterRampingUp) {
                    // Auto aim when shooter is ramping up - use thread for continuous updates
                    autoAim = true;

                    // Start auto aim thread if not already running
                    if (!autoAimController.isRunning()) {
                        autoAimController.startAutoAim();
                    }

                    // Get values for telemetry
                    double targetAngle = autoAimController.getTargetAngle();
                    double currentTurretAngle = autoAimController.getCurrentTurretHeading();
                    double aimError = autoAimController.getError();
                    double[] debugVals = autoAimController.getDebugValues();

                    // Check if aligned
                    isAligned = autoAimController.isOnTarget();

                    // Debug telemetry for auto aim
                    telemetry.addLine("=== AUTO AIM DEBUG ===");
                    telemetry.addData("Odo X", "%.2f", currentXOdo);
                    telemetry.addData("Odo Y", "%.2f", currentYOdo);
                    telemetry.addData("Odo Heading", "%.2f", currentHeadingOdo);
                    telemetry.addLine("--- Calculation ---");
                    telemetry.addData("AbsAngle (atan2)", "%.2f", debugVals[0]);
                    telemetry.addData("AdjustedHeading", "%.2f", debugVals[1]);
                    telemetry.addData("RelativeAngle", "%.2f", debugVals[2]);
                    telemetry.addData("TurretTarget", "%.2f", debugVals[3]);
                    telemetry.addLine("--- Result ---");
                    telemetry.addData("Target Angle", "%.2f", targetAngle);
                    telemetry.addData("Current Turret", "%.2f", currentTurretAngle);
                    telemetry.addData("Error", "%.2f", aimError);
                    telemetry.addData("Aligned", isAligned);

                    // Update current heading for manual control reference
                    currentHeading = currentTurretAngle;
                } else {
                    // No shooter input and no manual control - stop auto aim thread
                    if (autoAimController.isRunning()) {
                        autoAimController.stopAutoAim();
                    }
                    turretController.stopTurret();
                    autoAim = false;
                    isAligned = false;
                }
            } catch (Exception e) {
                telemetry.addData("Turret Error", e.getMessage());
            }

            if (gamepad2.dpad_down) {
                latchState = 0;
            } else if (gamepad2.dpad_up) {
                latchState = 1;
            }

            //light for shooter status
            try {
                double avgShooterVel = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;

                if (avgShooterVel > (shooterPower - shootingToleranceTeleOp) && avgShooterVel < (shooterPower + shootingToleranceTeleOp)) {
                    try {
                        light.setPosition(0.5);
                    } catch (Exception e) {
                        telemetry.addData("Light Error", e.getMessage());
                    }
                    if (((gamepad2.right_bumper && (gamepad2.right_trigger > 0.1)) && safeShooting) || autoShoot) {
                        try {
                            intake.setPower(1);
                        } catch (Exception e) {
                            telemetry.addData("Intake Error", e.getMessage());
                        }
                        latchState = 1;
                    }
                } else {
                    try {
                        light.setPosition(0.28);
                    } catch (Exception e) {
                        telemetry.addData("Light Error", e.getMessage());
                    }
                    if (((gamepad2.right_bumper && (gamepad2.right_trigger > 0.1)) && safeShooting) || autoShoot) {
                        try {
                            intake.setPower(0);
                        } catch (Exception e) {
                            telemetry.addData("Intake Error", e.getMessage());
                        }
                    }
                }
            } catch (Exception e) {
                telemetry.addData("Shooter Velocity Error", e.getMessage());
            }

            // Latch controls with error handling
            try {
                leftLatch.setPosition(1 - latchState);
            } catch (Exception e) {
                telemetry.addData("Latch Error", e.getMessage());
            }

            //Pose2D pos = odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);
            telemetry.addData("Robot X", currentXOdo);
            telemetry.addData("Robot Y", currentYOdo);
            telemetry.addData("Robot Heading", currentHeadingOdo);
            telemetry.addData("Auto Aim", autoAim);
            telemetry.addData("Auto Aim Aligned", isAligned);
            telemetry.addData("Auto Shoot", autoShoot);
            telemetry.addData("Safe Shooting", safeShooting);
            telemetry.addData("Turret Target Heading", currentHeading);
            telemetry.addData("Turret Actual Heading", autoAimController.getCurrentTurretHeading());
            telemetry.addData("Turret Power", turret.getPower());
            telemetry.addData("Left Shooter Velocity", leftShooter.getVelocity());
            telemetry.addData("Left Shooter Power", leftShooter.getPower());
            telemetry.addData("Right Shooter Velocity", rightShooter.getVelocity());
            telemetry.addData("Right Shooter Power", rightShooter.getPower());
            telemetry.update();
        }

        // Cleanup - stop all threads when OpMode ends
        autoAimController.stopAutoAim();
        shooter.stopVelocityPID();
    }
}
