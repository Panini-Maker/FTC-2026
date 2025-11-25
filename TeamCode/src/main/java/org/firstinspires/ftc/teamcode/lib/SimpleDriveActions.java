package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.idealVoltage;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class SimpleDriveActions {
    //declares hardware and variables
    public Telemetry telemetry;
    public DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public DcMotorEx shooterMotor;
    public GoBildaPinpointDriver odo;
    public DcMotor intake;
    public DcMotor transfer;
    public VoltageSensor voltageSensor;

    public double forward;
    public double right;
    public double rotate;
    public long timeouts_ms; //timeout in milliseconds
    public ElapsedTime runTime;

    //initializes hardware and variables for the DriveActions class
    public SimpleDriveActions(DcMotor frontLeftMotor, DcMotor frontRightMotor,
                              DcMotor backRightMotor, DcMotor backLeftMotor,
                              Telemetry telemetry, GoBildaPinpointDriver odo,
                              DcMotorEx shooterMotor, DcMotor intake,
                              DcMotor transfer, VoltageSensor voltageSensor,
                              ElapsedTime runTime) {

        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.telemetry = telemetry;
        this.odo = odo;
        this.shooterMotor = shooterMotor;
        this.runTime = runTime;
        this.intake = intake;
        this.transfer = transfer;
        this.voltageSensor = voltageSensor;
    }

    private double getVoltageCompensatedPower(double power) {
        double currentVoltage = voltageSensor.getVoltage();
        return power * (idealVoltage / currentVoltage);
    }

    //basic drive function, drives using power and time
    public void drive(double forward, double right, double rotate, long timeouts_ms) throws InterruptedException {
        this.forward = forward;
        this.right = right;
        this.rotate = rotate;
        this.timeouts_ms = timeouts_ms;
        double frontRightPower = forward + right + rotate;
        double frontLeftPower = forward - right - rotate;
        double backLeftPower = forward + right - rotate;
        double backRightPower = forward - right + rotate;
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)));
        // Change normalization only when maxPower > 1 to only need to do division when necessary.
        if (maxPower > 1) {
            frontRightPower /= maxPower;
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        frontRightMotor.setPower(frontRightPower * getVoltageCompensatedPower(1));
        frontLeftMotor.setPower(frontLeftPower * getVoltageCompensatedPower(1));
        backLeftMotor.setPower(backLeftPower * getVoltageCompensatedPower(1));
        backRightMotor.setPower(backRightPower * getVoltageCompensatedPower(1));

        sleep(timeouts_ms);
        stopMotor();
    }

    public void turnToHeadingWithOdo(double targetHeading, double power, double toleranceAngle, long timeoutMs) throws InterruptedException {
        runTime.reset();
        setMotorRunWithoutEncoder();
        odo.update();
        Pose2D pos;

        double currentHeading;
        double headingError;
        double rotationPower;
        //number of degrees off current heading is compared to target
        while (true) { //keeps running
            odo.update();
            pos = odo.getPosition();
            currentHeading = pos.getHeading(AngleUnit.DEGREES);
            headingError = targetHeading - currentHeading;
            rotationPower = (headingError / (Math.abs(headingError))) * power; //tune constant if needed

            drive(0, 0, rotationPower, 100);
            //if the heading error is less than ANGLE_TOLERANCE or time has ran out the robot will stop
            if ((Math.abs(headingError) < toleranceAngle) || (runTime.milliseconds() >= timeoutMs))
                break;
        }
        stopMotor();
    }

    // Stop all motors
    public void stopMotor() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    public void setMotorRunWithoutEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveToPosition(double forwardDistance, double rightDistance, double maxPower, double tolerance, double timeout_MS) throws InterruptedException {
        moveToPosition(forwardDistance, rightDistance, maxPower, tolerance, timeout_MS, false);
    }

    public void moveToPosition(double forwardDistance, double rightDistance, double maxPower, double tolerance, double timeout_MS, boolean runIntake) throws InterruptedException {
        odo.update(); // Reset odometry to start from the current position
        Pose2D pose;
        pose = odo.getPosition();

        double targetX = pose.getX(DistanceUnit.INCH) + rightDistance;
        double targetY = pose.getY(DistanceUnit.INCH) + forwardDistance;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        // Timeout in seconds

        while (timer.milliseconds() < timeout_MS) {
            odo.update();
            pose = odo.getPosition();

            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);

            double xError = targetX - currentX;
            double yError = targetY - currentY;

            // Calculate distance to the target position
            double distanceToTarget = Math.sqrt(xError * xError + yError * yError);

            // Break the loop if the robot is close enough to the target position
            if (distanceToTarget < tolerance) {
                stopMotor();
                break;
            }

            // Adjust proportional gains for forward and strafe movements
            double forwardPower = yError; // Proportional control for forward movement
            double strafePower = xError; // Proportional control for strafing

            // Normalize motor powers to ensure no motor exceeds the maximum power
            double maxCalculatedPower = Math.max(Math.max(forwardPower + strafePower, forwardPower - strafePower),
                    Math.max(-forwardPower - strafePower, -forwardPower + strafePower));

            forwardPower = (forwardPower / maxCalculatedPower) * maxPower;
            strafePower = (strafePower / maxCalculatedPower) * maxPower;

            // Set motor powers
            frontLeftMotor.setPower(forwardPower + strafePower);
            frontRightMotor.setPower(forwardPower - strafePower);
            backLeftMotor.setPower(forwardPower - strafePower);
            backRightMotor.setPower(forwardPower + strafePower);

            if (runIntake) {
                intake.setPower(0.8); // Run the intake motor
                transfer.setPower(0.3); // Run the transfer motor
            }

            sleep(100);

            // Debugging telemetry
            telemetry.addData("X Error", xError);
            telemetry.addData("Y Error", yError);
            telemetry.addData("X Position", currentX);
            telemetry.addData("Y Position", currentY);
            telemetry.addData("Rotation", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Time Elapsed", timer.seconds());
            telemetry.update();
        }

        // Stop all motors after reaching the position
        stopMotor();
        if (runIntake) {
            intake.setPower(0); // Stop the intake motor
            transfer.setPower(0); // Stop the transfer motor
        }
    }

    public void moveToPosition(double forwardDistance, double rightDistance,
                               double tolerance, double timeout_MS, boolean runIntake)
            throws InterruptedException {
        odo.update(); // Reset odometry to start from the current position
        Pose2D pose;
        pose = odo.getPosition();

        double maxPower;

        double targetX = pose.getX(DistanceUnit.INCH) + rightDistance;
        double targetY = pose.getY(DistanceUnit.INCH) + forwardDistance;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        // Timeout in seconds

        double totalDistance = Math.sqrt(forwardDistance * forwardDistance + rightDistance * rightDistance);
        double distanceThreshold = totalDistance * 0.2; // 20% of the total distance

        while (timer.milliseconds() < timeout_MS) {
            odo.update();
            pose = odo.getPosition();

            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);

            double xError = targetX - currentX;
            double yError = targetY - currentY;


            // Calculate distance to the target position
            double distanceToTarget = Math.sqrt(xError * xError + yError * yError);

            if (distanceThreshold >= distanceToTarget) {
                maxPower = 0.2;
            } else {
                maxPower = 0.8;
            }

            // Break the loop if the robot is close enough to the target position
            if (distanceToTarget < tolerance) {
                stopMotor();
                break;
            }

            // Adjust proportional gains for forward and strafe movements
            double forwardPower = yError; // Proportional control for forward movement
            double strafePower = xError; // Proportional control for strafing

            // Normalize motor powers to ensure no motor exceeds the maximum power
            double maxCalculatedPower = Math.max(Math.max(forwardPower + strafePower, forwardPower - strafePower),
                    Math.max(-forwardPower - strafePower, -forwardPower + strafePower));

            forwardPower = (forwardPower / maxCalculatedPower) * maxPower;
            strafePower = (strafePower / maxCalculatedPower) * maxPower;

            // Set motor powers
            frontLeftMotor.setPower(forwardPower + strafePower);
            frontRightMotor.setPower(forwardPower - strafePower);
            backLeftMotor.setPower(forwardPower - strafePower);
            backRightMotor.setPower(forwardPower + strafePower);

            if (runIntake) {
                intake.setPower(0.8); // Run the intake motor
                intake.setPower(0.2); // Run the intake motor
                transfer.setPower(0.3); // Run the transfer motor
            }

            sleep(100);

            // Debugging telemetry
            telemetry.addData("X Error", xError);
            telemetry.addData("Y Error", yError);
            telemetry.addData("X Position", currentX);
            telemetry.addData("Y Position", currentY);
            telemetry.addData("Rotation", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Time Elapsed", timer.seconds());
            telemetry.update();
        }

    }
}
