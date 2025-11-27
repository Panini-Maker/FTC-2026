package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingSlowDownSpeed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShootingAction {
    public DcMotorEx shooterMotor;
    public DcMotor transfer;
    public DcMotor intake;
    public ShootingAction(DcMotorEx shooterMotor, DcMotor transfer, DcMotor intake) {
        this.shooterMotor = shooterMotor;
        this.transfer = transfer;
        this.intake = intake;
    }

    public void shoot(int shooterVelocity, int shootDurationMs, int rampUpTimeMs, boolean slowDown) throws InterruptedException {
        shooterMotor.setVelocity(shooterVelocity);
        Thread.sleep(rampUpTimeMs);
        transfer.setPower(0.7);
        intake.setPower(0.7);
        if (slowDown) {
            Thread.sleep(shootDurationMs / 3);
            shooterMotor.setVelocity(shooterVelocity - shootingSlowDownSpeed); //NOTE: set intake-transfer to 0 for slowdown
            transfer.setPower(0);
            intake.setPower(0);
            Thread.sleep(500);
            transfer.setPower(0.7);
            intake.setPower(0.7);
            Thread.sleep((shootDurationMs * 2L) / 3);
        } else {
            Thread.sleep(shootDurationMs);
        }

        transfer.setPower(0);
        intake.setPower(0);
        shooterMotor.setPower(0);
    }
}
