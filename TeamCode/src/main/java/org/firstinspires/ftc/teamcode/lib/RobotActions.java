package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class RobotActions {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;

    private final DcMotorEx turret;

    private final DcMotor intake;

    private final Servo rightLatch;
    private final Servo leftLatch;
    private final Servo hood;

    private final Servo light;

    public RobotActions(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,
                        DcMotorEx rightShooter, DcMotorEx leftShooter, DcMotorEx turret, DcMotor intake,
                        Servo rightLatch, Servo leftLatch, Servo hood, Servo light) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.turret = turret;
        this.intake = intake;
        this.rightLatch = rightLatch;
        this.leftLatch = leftLatch;
        this.hood = hood;
        this.light = light;

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void driveRobotCentric(double y, double x, double rx, double power) {
        double frontLeftPower = (y + x + rx) / Math.max(Math.abs(y + x + rx), 1) * power;
        double frontRightPower = (y - x - rx) / Math.max(Math.abs(y - x - rx), 1) * power;
        double backLeftPower = (y - x + rx) / Math.max(Math.abs(y - x + rx), 1) * power;
        double backRightPower = (y + x - rx) / Math.max(Math.abs(y + x - rx), 1) * power;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void driveFieldCentric(Pose2D pos, double y, double x, double rx, double power) {
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX  * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator * power;
        double backLeftPower = (rotY - rotX + rx) / denominator * power;
        double frontRightPower = (rotY - rotX - rx) / denominator * power;
        double backRightPower = (rotY + rotX - rx) / denominator * power;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void setLatch(double pos) {
        rightLatch.setPosition(pos);
        leftLatch.setPosition(1 - pos);
    }

    public double getDistanceFromGoal(Pose2D pos, boolean targetIsRed) {

        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double goal_pos_x;
        double goal_pos_y;

        if (targetIsRed) {
            goal_pos_x = 72;
            goal_pos_y = 72;
        } else {
            goal_pos_x = -72;
            goal_pos_y = 72;
        }

        double distance = Math.sqrt(Math.pow((goal_pos_x-x), 2) + Math.pow((goal_pos_y-y), 2));

        return distance;
    }

    public double getShooterRPM(double distance) {
        return distance * 7.05 + 1019;
    }

    public double getShooterAngle(double distance) {
        return 0.061 + 0.00602 * distance - 0.0000207 * distance * distance;
    }

    public double angleToGoal(Pose2D pos, double turretCurrentAngle, boolean targetIsRed) {
        // Get robot x, y, heading
        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double heading = pos.getHeading(AngleUnit.DEGREES);

        // find goal coordinates
        double goal_x;
        double goal_y;

        if (targetIsRed) {
            goal_x = 72;
            goal_y = 72;
        } else {
            goal_x = -72;
            goal_y = 72;
        }

        double dist_x = goal_x - x;
        double dist_y = goal_y - y;

        double fieldAngle = - Math.toDegrees(Math.atan(dist_x/dist_y)); // Absolute angle
        double robotAngle = fieldAngle - heading; // Angle needed to turn to (without turret)
        double turretAngle = robotAngle - turretCurrentAngle;

        turretAngle = setAngleInRange(turretAngle);

        return turretAngle;
    }

    public double setAngleInRange(double angle) {
        double output = angle;
        if (angle > 180) {
            while (output > 180) {
                output -= 180;
            }
        } else if (angle < 180) {
            while (output < -180) {
                output += 180;
            }
        }
        return output;
    }
}
