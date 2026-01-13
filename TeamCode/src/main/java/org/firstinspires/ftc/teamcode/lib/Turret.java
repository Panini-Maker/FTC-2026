package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.turretTicksPerDegree;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    private DcMotorEx turret;
    private final double ticksPerDegree = turretTicksPerDegree;
    public Turret(DcMotorEx turret) {
        this.turret = turret;
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder at initialization
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Set to use encoder
    }

    public void spinToPosition (int position, double power) {
        turret.setTargetPosition(position);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }

    public double spinToHeading (double headingDegrees, double power) {
        int currentPosition = turret.getCurrentPosition();
        int targetPosition = (int)(headingDegrees * ticksPerDegree);// - currentPosition; // Calculate relative target position
        spinToPosition(targetPosition, power);
        currentPosition = turret.getCurrentPosition();

        // Debugging telemetry
        System.out.println("Target Position (ticks): " + targetPosition);
        System.out.println("Current Position (ticks): " + currentPosition);
        System.out.println("Ticks per Degree: " + ticksPerDegree);

        return (currentPosition / ticksPerDegree);
    }

    public void stopTurret() {
        turret.setPower(0);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
