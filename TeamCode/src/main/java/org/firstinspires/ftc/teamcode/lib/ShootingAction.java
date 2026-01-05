package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingSlowDownSpeed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ShootingAction {
    public DcMotorEx leftShooter, rightShooter;
    public DcMotor intake;
    public DcMotor turret;
    public Servo hoodServo, leftLatch, rightLatch;

    public ShootingAction(DcMotorEx leftShooter, DcMotorEx rightShooter, DcMotor intake, DcMotor turret,
                          Servo hoodServo, Servo leftLatch, Servo rightLatch) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.intake = intake;
        this.turret = turret;
        this.hoodServo = hoodServo;
        this.leftLatch = leftLatch;
        this.rightLatch = rightLatch;
    }

    public void shoot(int shooterVelocity, int shootDurationMs, int rampUpTimeMs, boolean slowDown) throws InterruptedException {
        leftShooter.setVelocity(shooterVelocity);
        rightShooter.setVelocity(shooterVelocity);
        Thread.sleep(rampUpTimeMs);
        leftLatch.setPosition(1); // Open latches
        rightLatch.setPosition(0);
        intake.setPower(0.7);
        if (slowDown) {
            Thread.sleep(shootDurationMs / 3);
            leftShooter.setVelocity(shooterVelocity - shootingSlowDownSpeed); //NOTE: set intake-transfer to 0 for slowdown
            rightShooter.setVelocity(shooterVelocity - shootingSlowDownSpeed);
            intake.setPower(0);
            Thread.sleep(500);
            intake.setPower(0.7);
            Thread.sleep((shootDurationMs * 2L) / 3);
        } else {
            Thread.sleep(shootDurationMs);
        }

        intake.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        leftLatch.setPosition(0); // Close latches
        rightLatch.setPosition(1);
    }
}
