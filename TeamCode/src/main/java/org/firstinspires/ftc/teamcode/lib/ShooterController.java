package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterController {

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final PIDController pidController;
    private final Telemetry telemetry;

    public ShooterController(DcMotorEx leftShooter, DcMotorEx rightShooter, double Kp, double Ki, double Kd, Telemetry telemetry) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.telemetry = telemetry;

        // Initialize PID controller
        this.pidController = new PIDController(Kp, Ki, Kd, telemetry);

        // Set motor directions if needed
        this.leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Runs the shooter motors at the desired velocity using PID control.
     *
     * @param desiredVelocity The target velocity in ticks per second.
     */
    public void runShooter(double desiredVelocity) {
        double currentVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
        double powerAdjustment = pidController.calculate(desiredVelocity, currentVelocity);

        leftShooter.setPower(powerAdjustment);
        rightShooter.setPower(powerAdjustment);

        //telemetry.addData("Shooter Power Adjustment", powerAdjustment);
        //telemetry.update();
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        pidController.reset();
    }

    /**
     * Resets the PID controller.
     */
    public void resetPID() {
        pidController.reset();
    }

    /**
     * Dynamically update PID constants.
     */
    public void setPIDConstants(double Kp, double Ki, double Kd) {
        pidController.setKp(Kp);
        pidController.setKi(Ki);
        pidController.setKd(Kd);
    }
}
