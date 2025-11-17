package org.firstinspires.ftc.teamcode.lib;

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

    public void shoot(int shooterVelocity, int shootDurationMs, int transferDelayMs) throws InterruptedException {
        shooterMotor.setVelocity(shooterVelocity);
        Thread.sleep(transferDelayMs);
        transfer.setPower(0.8);
        intake.setPower(0.8);
        Thread.sleep(shootDurationMs);
        transfer.setPower(0);
        intake.setPower(0);
        shooterMotor.setPower(0);
    }
}
