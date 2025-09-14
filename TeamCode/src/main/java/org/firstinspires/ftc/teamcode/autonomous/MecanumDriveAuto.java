package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
public class MecanumDriveAuto {
    public void CalcSpeeds(int forward, int strafe,
                               int rotate, double multiplier,
                               DcMotor frontLeft, DcMotor frontRight,
                               DcMotor backLeft, DcMotor backRight) {

        //May have to multiply strafe by 1.1 to counteract imperfect strafing

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) +
                Math.abs(rotate), 1);
        double frontLeftPower = (forward - strafe - rotate) * multiplier / denominator;
        double backLeftPower = (forward - strafe + rotate) * multiplier / denominator;
        double frontRightPower = (forward + strafe + rotate) * multiplier / denominator;
        double backRightPower = (forward + strafe - rotate) * multiplier / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}
