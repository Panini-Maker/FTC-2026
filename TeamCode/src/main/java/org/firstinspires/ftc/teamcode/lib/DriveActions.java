package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class DriveActions {
    //declares hardware and variables
    protected Servo sampleClaw, wrist;
    //public BHI260IMU imu;
    public ElapsedTime runtime2 = new ElapsedTime();
    public ElapsedTime runtime1 = new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime();
    public Telemetry telemetry;
    public DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public DcMotorEx leftExtension, rightExtension, armMotor;
    public BHI260IMU imu;
    public ElapsedTime runtime;
    GoBildaPinpointDriver odo;

    public double forward;
    public double right;
    public double rotate;
    public long timeouts_ms; //timeout in milliseconds
    protected int ANGLE_TOLERANCE;
    protected double MIN_MOTOR_POWER = 0.2;
    double COUNTS_PER_INCH = 40.5; //the number of encoder ticks per inch the wheel travels

    //initializes hardware and variables for the DriveActions class
    public DriveActions(DcMotorEx frontLeftMotor, DcMotorEx frontRightMotor,
                        DcMotorEx backRightMotor, DcMotorEx backLeftMotor,
                        DcMotorEx leftExtension, DcMotorEx rightExtension,
                        DcMotorEx armMotor, Servo sampleClaw, Servo wrist, BHI260IMU imu,
                        Telemetry telemetry, ElapsedTime runtime, GoBildaPinpointDriver odo, int ANGLE_TOLERANCE, double MIN_MOTOR_POWER) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.leftExtension = leftExtension;
        this.rightExtension = rightExtension;
        this.armMotor = armMotor;
        this.sampleClaw = sampleClaw;
        this.runtime = runtime;
        this.imu = imu;
        this.telemetry = telemetry;
        this.ANGLE_TOLERANCE = ANGLE_TOLERANCE;
        this.MIN_MOTOR_POWER = MIN_MOTOR_POWER;
        this.odo = odo;

        imu.initialize();
    }

    //basic drive function, drives using power and time
    public void drive(double forward, double right, double rotate, long timeouts_ms) {
        runtime1.reset();
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
        frontRightMotor.setPower(frontRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
//        if (maxPower < 1) {
//            maxPower = 1;
//        }
//        frontRightMotor.setPower(frontRightPower/maxPower);
//        frontLeftMotor.setPower(frontLeftPower/maxPower);
//        backLeftMotor.setPower(backLeftPower/maxPower);
//        backRightMotor.setPower(backRightPower/maxPower);
        if (timeouts_ms > 0) { //if timeouts_ms = 0, the wheels will keep running
            while (runtime1.milliseconds() < timeouts_ms) {
                //Optionally, check if encoder position is reached
                if (runtime1.milliseconds() >= timeouts_ms) { //add or condition for encoder position
                    stopMotor();
                    break;
                }
            }
        }

    }

    // Stop all motors
    public void stopMotor() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    //moves the robot a certain distance in inches  forward, backward, or strafing
    public void moveDistance(double distanceInches, double power, int tolerance, long timeoutMs, boolean isStrafe) {
        int targetCounts = (int) (distanceInches * COUNTS_PER_INCH);
        //int targetCounts = 122;
        power = Math.abs(power);
        resetEncoders();
        setWheelTolerance(tolerance);
        double targetHeading = getCurrentHeading();
        //sets target positions

        if (isStrafe) {
            frontLeftMotor.setTargetPosition(-targetCounts);
            frontRightMotor.setTargetPosition(targetCounts);
            backLeftMotor.setTargetPosition(targetCounts);
            backRightMotor.setTargetPosition(-targetCounts);
        } else {

            frontLeftMotor.setTargetPosition(targetCounts);
            frontRightMotor.setTargetPosition(targetCounts);
            backLeftMotor.setTargetPosition(targetCounts);
            backRightMotor.setTargetPosition(targetCounts);
        }

        runtime2.reset();
        //resets runtime to start measuring time when the motors turn
        setMotorsRunToPosition();
//        while (runtime2.milliseconds() < timeoutMs) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        sleep(timeoutMs);
//
//            //stop all motors if the motors are not busy
//            if (!areMotorsBusy()) {
//                stopMotor();
//                break;
//            }
//
//        }
    }

    //turn to a specific direction using the IMU
    public void turnToHeadingWithIMU(double targetHeading, double power, long timeoutMs) {
        runtime2.reset();
        resetEncoders();
        setMotorRunwithoutEncoder();
        power = Math.abs(power);
        double currentHeading = getCurrentHeading();
        double headingError = targetHeading - currentHeading;
        double rotationPower;
        //number of degrees off current heading is compared to target
        while (true) { //keeps running
            currentHeading = getCurrentHeading();
            headingError = targetHeading - currentHeading;
            rotationPower = -0.1 * headingError; //tune constant if needed
            double sign_power = Math.signum(rotationPower);
            //signum determines the sign of a number
            //It returns 1 if positive, 0 if zero, and -1 if negative
            if (Math.abs(rotationPower) < MIN_MOTOR_POWER) {
                rotationPower = MIN_MOTOR_POWER * sign_power;
            } else if (Math.abs(rotationPower) > power) {
                rotationPower = sign_power * power;
            }
            drive(0, 0, rotationPower, 0);
            //if the heading error is less than ANGLE_TOLERANCE or time has ran out the robot will stop
            if ((Math.abs(headingError) < ANGLE_TOLERANCE) || (runtime2.milliseconds() >= timeoutMs))
                break;
        }
        stopMotor();
    }
    public void turnToHeadingWithOdo2(double targetHeading, double power, double toleranceAngle, long timeoutMs) {
        runtime2.reset();
        setMotorRunwithoutEncoder();
        power = Math.abs(power);
        odo.update();
        Pose2D pos = odo.getPosition();

        double currentHeading;
        double headingError;
        double rotationPower;
        //number of degrees off current heading is compared to target
        while (true) { //keeps running
            odo.update();
            pos = odo.getPosition();
            currentHeading = pos.getHeading(AngleUnit.DEGREES);
            headingError = targetHeading - currentHeading;
            rotationPower = -0.1 * headingError; //tune constant if needed
            double sign_power = Math.signum(rotationPower);
            //signum determines the sign of a number
            //It returns 1 if positive, 0 if zero, and -1 if negative
            if (Math.abs(rotationPower) < MIN_MOTOR_POWER) {
                rotationPower = MIN_MOTOR_POWER * sign_power;
            } else if (Math.abs(rotationPower) > power) {
                rotationPower = sign_power * power;
            }
            drive(0, 0, rotationPower, 0);
            //if the heading error is less than ANGLE_TOLERANCE or time has ran out the robot will stop
            if ((Math.abs(headingError) < toleranceAngle) || (runtime2.milliseconds() >= timeoutMs))
                break;
        }
        stopMotor();
    }

    public void turnToHeadingWithOdometry(double targetHeading, double power, double toleranceAngle, long timeoutMs) {
        runtime2.reset();
        setMotorRunwithoutEncoder();
        power = Math.abs(power);
        odo.update();
        Pose2D pos = odo.getPosition();

        double currentHeading;
        double headingError;
        double rotationPower;
        double original_error = targetHeading - pos.getHeading(AngleUnit.DEGREES);
        // Return if error is within angle tolerance
        telemetry.addData("original angle error", original_error);
        if (Math.abs(original_error) < toleranceAngle) {
            return;
        }
        original_error = Math.toRadians(original_error);
        while (true) { //keeps running
            odo.update();
            pos = odo.getPosition();
            currentHeading = pos.getHeading(AngleUnit.DEGREES);
            headingError = targetHeading - currentHeading;
            if ((Math.abs(headingError) < toleranceAngle) || (runtime2.milliseconds() >= timeoutMs))
                break;
            // wrap error to (-pi, pi)
            headingError = Math.toRadians(headingError);
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
            double scale = trapezoidProfile(headingError, original_error, 0.3, 0.9);

            rotationPower = - scale * power * Math.signum(headingError);
            rotationPower = Math.min(Math.abs(rotationPower), 0.15) * Math.signum(rotationPower);
            drive(0, 0, rotationPower, 0);
            //if the heading error is less than ANGLE_TOLERANCE or time has ran out the robot will stop

        }
        stopMotor();
    }

    //turns the robot a certain amount of degrees
    public void turnByAngle(double angle, double power, long timeoutMs) {
        double targetHeading = getCurrentHeading() + angle;
        turnToHeadingWithIMU(targetHeading, power, timeoutMs);
    }

    //gets the average of 5 readings from the distance sensor for more accuracy
    /*
    public double getAverageDistance(boolean isStrafe) {
        double reading1, reading2, reading3, reading4, reading5;
        double average;
        if (isStrafe) {
            reading1 = rightDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading2 = rightDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading3 = rightDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading4 = rightDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading5 = rightDistance.getDistance(DistanceUnit.INCH);
            average = (reading1 + reading2 + reading3 + reading4 + reading5) / 5.0;
            return average;
        } else {
            reading1 = frontDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading2 = frontDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading3 = frontDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading4 = frontDistance.getDistance(DistanceUnit.INCH);
            sleep(10);
            reading5 = frontDistance.getDistance(DistanceUnit.INCH);
            average = (reading1 + reading2 + reading3 + reading4 + reading5) / 5.0;
            return average;
        }
    }

     */

    /*
    //moves the robot to a certain distance away from the wall in inches
    //isStrafe determines if the front or right distance sensor is used
    public void moveToDistance(double targetDistanceInches, double power, long timeoutMs, int heading, int tolerance, boolean isStrafe) {
        turnToHeadingWithIMU(heading, power, timeoutMs);
        if (!isStrafe) {
            //double currentDistance = getAverageDistance(false);
            double currentDistance = frontDistance.getDistance(DistanceUnit.INCH);
            telemetry.addData("CURRENT DISTANCE", currentDistance);
            moveDistance(-(currentDistance - targetDistanceInches), power, tolerance, timeoutMs, false);
        } else {
            double currentDistance = getAverageDistance(true);
            moveDistance(currentDistance - targetDistanceInches, power, tolerance, timeoutMs, true);
        }
    }

     */

/*    public int calculateForwardAndRightMovement (boolean use_targetX, double error, boolean isStrafe) {
        TBD
    }*/

    public void movetoPosition_Odo_PID (boolean use_targetX, Pose2D poseTarget, double power, int right, int forward, double toleranceInches, double toleranceDegrees, long timeouts_ms){
        if (right * forward != 0) {
            throw new IllegalMonitorStateException("Cannot move forward and strafe together");
        }
        double maxPower = Math.abs(power);
        runtime.reset();
        timer.reset();
        final double kp = 0.05;
        final double ki = 0.5;
        final double kd = 0.002;
        double previousError = 0;
        double integral = 0;
        double derivative = 0;
        double proportional = 0;
        double currentX, currentY, currentHeading;
        double error;
        Pose2D pose;
        setMotorRunwithoutEncoder();
        odo.update();
        pose = odo.getPosition();
        double targetHeading = pose.getHeading(AngleUnit.DEGREES); // get current heading robot move but keep the same heading
        while (runtime.milliseconds() < timeouts_ms){
            double dt = timer.seconds();
            timer.reset();
            odo.update();
            pose = odo.getPosition();
            currentX = pose.getX(DistanceUnit.INCH);
            currentY = pose.getY(DistanceUnit.INCH);
            currentHeading = pose.getHeading(AngleUnit.DEGREES);
            if (use_targetX) {
                error = poseTarget.getX(DistanceUnit.INCH) - currentX;
            }
            else {
                error = poseTarget.getY(DistanceUnit.INCH) - currentY;
            }

            double headingError = targetHeading - currentHeading;
            if ((Math.abs(error) < toleranceInches) && (Math.abs(headingError) < toleranceDegrees)){
                stopMotor();
                telemetry.addData("Arrived target destination", 0);
                telemetry.update();
                break;
            }
            else {
                telemetry.addData("Moving to target destination", 0);
            }

            if (Math.abs(error) < 10) {
                power = Math.max(Math.min(Math.abs(error)*0.1, Math.abs(power)), 0.4);
            }

            double forward_power = 0;
            double strafe_power = 0;
            if (right == 1) {
                strafe_power = Math.abs(power);
            }
            else if (right == 0) {
                if (forward == 1) {
                    forward_power = -Math.abs(power);
                }
                else if (forward == -1) {
                    forward_power = Math.abs(power);
                }
            }
            else {
                strafe_power = -Math.abs(power);
            }

            // double xPower = xPID.calculate(xError);
            double headingPower = 0;
            integral += ki * (headingError * dt);
            integral = Math.signum(integral) * Math.min(Math.abs(integral), 0.1);
            derivative = kd * (headingError - previousError) / dt;
            derivative = Math.signum(derivative) * Math.min(Math.abs(derivative), 0.1);
            proportional = kp * headingError;
            proportional = Math.signum(proportional) * Math.min(Math.abs(proportional), 0.1);
            previousError = headingError;
            headingPower =  proportional + integral + derivative;

            /*
            telemetry.addData("Forward power", forward_power);
            telemetry.addData("Strafe power", strafe_power);
            telemetry.addData("Target heading (rad)", targetHeading/180*Math.PI);
            telemetry.addData("Odometry X Y position (in)", currentX + " " + currentY);
            telemetry.addData("Odometry heading (rad)", currentHeading/180*Math.PI);
            telemetry.addData("heading error (degree)", headingError);
            telemetry.addData("error", error);
            telemetry.addData("main power", power);
            telemetry.addData("proportional", proportional);
            telemetry.addData("derivative", derivative);
            telemetry.addData("integral", integral);
            telemetry.addData("dt", dt);
            telemetry.addData("heading power", headingPower);
            telemetry.update();

             */
            drive(forward_power, strafe_power, -headingPower, 0);

        }
        stopMotor();
    }

    public void movetoPosition_Odo_PID2 (Pose2D poseTarget, double power, double toleranceInches,
                                         double toleranceDegrees, long timeouts_ms){
        double maxPower = Math.abs(power);
        runtime.reset();
        timer.reset();
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double previousError = 0;
        double integral = 0;
        double derivative = 0;
        double proportional = 0;
        double currentX, currentY, currentHeading;
        double x_error;
        double y_error;
        Pose2D pose;
        setMotorRunwithoutEncoder();
        odo.update();
        pose = odo.getPosition();
        double targetHeading = pose.getHeading(AngleUnit.DEGREES); // get current heading robot move but keep the same heading
        double original_x_error = poseTarget.getX(DistanceUnit.INCH) - pose.getX(DistanceUnit.INCH);;
        double original_y_error = poseTarget.getY(DistanceUnit.INCH) - pose.getY(DistanceUnit.INCH);;
        double original_error = Math.sqrt((original_x_error * original_x_error +
                original_y_error * original_y_error));
        telemetry.addData("Original error", original_x_error);
        telemetry.addData("x error", original_x_error);
        telemetry.addData("y error", original_y_error);
        if (Math.abs(original_error) < toleranceInches) {
            return;
        }
        while (runtime.milliseconds() < timeouts_ms){
            double dt = timer.seconds();
            timer.reset();
            odo.update();
            pose = odo.getPosition();
            currentX = pose.getX(DistanceUnit.INCH);
            currentY = pose.getY(DistanceUnit.INCH);
            currentHeading = pose.getHeading(AngleUnit.DEGREES);
            x_error = poseTarget.getX(DistanceUnit.INCH) - currentX;
            y_error = poseTarget.getY(DistanceUnit.INCH) - currentY;
            double distance = Math.sqrt(x_error * x_error + y_error * y_error);
            double headingError = targetHeading - currentHeading;

            if ((distance < toleranceInches) && (Math.abs(headingError) < toleranceDegrees)){
                stopMotor();
                telemetry.addData("Arrived target destination", 0);
                telemetry.update();
                break;
            }
            else {
                telemetry.addData("Moving to target destination", 0);
            }
            // Move at angle
            double angle = Math.atan2(x_error, y_error);

            // Set minimum power to 0.15, change it if needed when testing
            // Trapezoid power as robot move closer to target
            double scale = trapezoidProfile(distance, original_error, 0.3, 0.9);
            double forward_power = - Math.cos(Math.toRadians(currentHeading) - angle);
            double strafe_power = Math.sin(Math.toRadians(currentHeading) - angle);
            // Normalize
            double max_scale = Math.max(Math.abs(forward_power), Math.abs(strafe_power));
            forward_power = forward_power/max_scale * scale * power;
            strafe_power = strafe_power/max_scale * scale * power;
            strafe_power = Math.max(Math.abs(strafe_power), 0.15) * Math.signum(strafe_power);
            forward_power = Math.max(Math.abs(forward_power), 0.15) * Math.signum(forward_power);


            // double xPower = xPID.calculate(xError);
            double headingPower = 0;
            headingError = Math.toRadians(headingError);
            // wrap to (-pi, pi)
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
            integral += ki * (headingError * dt);
//            integral = Math.signum(integral) * Math.min(Math.abs(integral), 0.1);
            derivative = kd * (headingError - previousError) / dt;
//            derivative = Math.signum(derivative) * Math.min(Math.abs(derivative), 0.1);
            proportional = kp * headingError;
//            proportional = Math.signum(proportional) * Math.min(Math.abs(proportional), 0.1);
            previousError = headingError;
            headingPower =  proportional + integral + derivative;
            headingPower = Math.signum(headingPower) * Math.max(Math.abs(headingPower), 0.2);

            /*
            telemetry.addData("Forward power", forward_power);
            telemetry.addData("Strafe power", strafe_power);
            telemetry.addData("Target heading (rad)", targetHeading/180*Math.PI);
            telemetry.addData("Odometry X Y position (in)", currentX + " " + currentY);
            telemetry.addData("Odometry heading (rad)", currentHeading/180*Math.PI);
            telemetry.addData("heading error (degree)", headingError);
            telemetry.addData("error", error);
            telemetry.addData("main power", power);
            telemetry.addData("proportional", proportional);
            telemetry.addData("derivative", derivative);
            telemetry.addData("integral", integral);
            telemetry.addData("dt", dt);
            telemetry.addData("heading power", headingPower);
            telemetry.update();

             */
            drive(forward_power, strafe_power, -headingPower, 0);

        }
        stopMotor();
    }

    public static double trapezoidProfile(double current_error, double max_error,
                                          double alpha, double beta) {
        //remember to multiply with max power
        // 0 < alpha < beta < 1, alpha for rising edge and beta for falling edge
        if (alpha <= 0 || alpha > 1 || beta < 0 || beta >= 1 || alpha >= beta) {
            return 0;
        }
        max_error = Math.abs(max_error);
        if (max_error == 0) {
            throw new IllegalArgumentException("max_error input must be positive");
        }
        if (Math.abs(current_error) > max_error) {
            current_error = max_error * Math.signum(current_error);
        }
        double normalize = Math.abs(current_error)/ max_error;

        if (normalize<alpha) {
            return normalize/alpha;
        }
        else if (normalize<= beta) {
            return 1.0;
        }
        else {
            return (1-normalize)/(1-beta);
        }
    }

    public double wrapAngle(double angle) {
        double out_angle = 0.0;
        while (angle > 180) out_angle -= 360;
        while (angle <= 180) out_angle += 360;
        return out_angle;
    }

    //resets the encoders to 0 for the wheel motors
    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //sets the runmode of the wheels to run to the target position
    //the sign(+ or -) of the power given does not matter
    public void setMotorsRunToPosition() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //sets the runmode to run using encoder
    public void setMotorRunUsingEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //sets the runmode to run without encoder
    //it will run using a given power and time
    public void setMotorRunwithoutEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //gets the current heading compared to the init heading
    public double getCurrentHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    //checks if the motors are busy
    //only returns false if all are free
    private boolean areMotorsBusy() {
        return frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy();
    }

    //sets the tolerance for the target position for the wheels
    public void setWheelTolerance(int tolerance) {
        frontLeftMotor.setTargetPositionTolerance(tolerance);
        frontRightMotor.setTargetPositionTolerance(tolerance);
        backLeftMotor.setTargetPositionTolerance(tolerance);
        backRightMotor.setTargetPositionTolerance(tolerance);
    }

    //moves the extension given power
    public void raiseExtension(double power) {
        rightExtension.setPower(power);
        leftExtension.setPower(power);
    }

    //stops extensions
    //-0.02 power prevents the extension from slipping
    public void stopExtension() {
        rightExtension.setPower(-0.02);
        leftExtension.setPower(-0.02);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //moves the arm to a target position
    public void moveArm(int target) {
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1); // Provide some power for movement
    }

    //stops the arm motor
    public void stopArm() {
        armMotor.setPower(0); // Immediately stop the arm motor
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset motor mode to avoid interference
    }

    //moves the claw to a specific position
    public void moveClaw(double clawPosition) {
        sampleClaw.setPosition(clawPosition);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void movetoPosition_Odo_PID_3 (boolean use_targetX, Pose2D poseTarget, double power, int right, int forward, double toleranceInches, double toleranceDegrees, long timeouts_ms){
        if (right * forward != 0) {
            throw new IllegalMonitorStateException("Cannot move forward and strafe together");
        }
        double maxPower = Math.abs(power);
        runtime.reset();
        timer.reset();
        final double kp = 0.05;
        final double ki = 0.5;
        final double kd = 0.002;
        double previousError = 0;
        double integral = 0;
        double derivative = 0;
        double proportional = 0;
        double currentX, currentY, currentHeading;
        double error;
        Pose2D pose;
        setMotorRunwithoutEncoder();
        odo.update();
        pose = odo.getPosition();
        double targetHeading = pose.getHeading(AngleUnit.DEGREES); // get current heading robot move but keep the same heading
        while (runtime.milliseconds() < timeouts_ms){
            double dt = timer.seconds();
            timer.reset();
            odo.update();
            pose = odo.getPosition();
            currentX = pose.getX(DistanceUnit.INCH);
            currentY = pose.getY(DistanceUnit.INCH);
            currentHeading = pose.getHeading(AngleUnit.DEGREES);
            if (use_targetX) {
                error = poseTarget.getX(DistanceUnit.INCH) - currentX;
            }
            else {
                error = poseTarget.getY(DistanceUnit.INCH) - currentY;
            }

            double headingError = targetHeading - currentHeading;
            if ((Math.abs(error) < toleranceInches) && (Math.abs(headingError) < toleranceDegrees)){
                stopMotor();
                telemetry.addData("Arrived target destination", 0);
                telemetry.update();
                break;
            }
            else {
                telemetry.addData("Moving to target destination", 0);
            }

            if (Math.abs(error) < 10) {
                power = Math.max(Math.min(Math.abs(error)*0.1, Math.abs(power)), 0.4);
            }

            double forward_power = 0;
            double strafe_power = 0;
            if (right == 1) {
                strafe_power = Math.abs(power);
            }
            else if (right == 0) {
                if (forward == 1) {
                    forward_power = -Math.abs(power);
                }
                else if (forward == -1) {
                    forward_power = Math.abs(power);
                }
            }
            else {
                strafe_power = -Math.abs(power);
            }

            // double xPower = xPID.calculate(xError);
            double headingPower = 0;
            /*
            integral += ki * (headingError * dt);
            integral = Math.signum(integral) * Math.min(Math.abs(integral), 0.1);
            derivative = kd * (headingError - previousError) / dt;
            derivative = Math.signum(derivative) * Math.min(Math.abs(derivative), 0.1);
            proportional = kp * headingError;
            proportional = Math.signum(proportional) * Math.min(Math.abs(proportional), 0.1);
            previousError = headingError;
            headingPower =  proportional + integral + derivative;

             */

            /*
            telemetry.addData("Forward power", forward_power);
            telemetry.addData("Strafe power", strafe_power);
            telemetry.addData("Target heading (rad)", targetHeading/180*Math.PI);
            telemetry.addData("Odometry X Y position (in)", currentX + " " + currentY);
            telemetry.addData("Odometry heading (rad)", currentHeading/180*Math.PI);
            telemetry.addData("heading error (degree)", headingError);
            telemetry.addData("error", error);
            telemetry.addData("main power", power);
            telemetry.addData("proportional", proportional);
            telemetry.addData("derivative", derivative);
            telemetry.addData("integral", integral);
            telemetry.addData("dt", dt);
            telemetry.addData("heading power", headingPower);
            telemetry.update();

             */
            drive(forward_power, strafe_power, -headingPower, 0);

        }
        stopMotor();
    }
}