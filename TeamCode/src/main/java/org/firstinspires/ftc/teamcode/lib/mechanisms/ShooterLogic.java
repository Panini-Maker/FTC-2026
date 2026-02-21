package org.firstinspires.ftc.teamcode.lib.mechanisms;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKd;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKi;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shooterKp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ShooterController;


public class ShooterLogic {

    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;

    private DcMotorEx turret;

    private DcMotor intake;

    private Telemetry telemetry;

    private Servo hood;
    private Servo leftLatch;
    private ShooterController shooter = new ShooterController(leftShooter, rightShooter, shooterKp, shooterKi, shooterKd, telemetry);

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum ShooterState {
        IDLE,
        SPIN_UP,
        SHOOT,
        INTAKE
    }

    private double latchState = 0;

    private void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        leftLatch = hardwareMap.get(Servo.class, "leftLatch");

    }
}
