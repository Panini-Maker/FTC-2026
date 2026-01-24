package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.Vector2d;

public class TuningVars {
    public static int odoXOffset = 75; // in mm
    public static int odoYOffset = -146; // in mm
    public static double shooterKp = 0.005; //0.0075
    public static double shooterKi = 0.0011; //0.0009
    public static double shooterKd = 0; //0.0003
    public static int shootingSlowDownSpeed = 200;
    public static int shotgun = 1550; //Was 1600
    public static int shotgunTeleOp = 1470;
    public static int sniper = 2000; //Was 1550
    public static int sniperAuto = 1900; // Was 2100 but shooter cannot reach that speed reliably, Changed to 1950
    // Was 1950, but not enough for autonomous shots
    // Shots too high at 1976 rpm, 1925 causes problem where it hits backboard sometimes
    public static int idle = 800; // Was 1200
    public static int shootingToleranceAuto = 5; // in RPM
    public static int shootingToleranceTeleOp = 50; // in RPM
    public static int shootDurationMs = 2500; // in milliseconds
    public static int rampUpTime = 3000; // in milliseconds
    public static int idealVoltage = 12; // in volts
    public static double turretMotorTPR = 537.7; // ticks per revolution
    public static double turretMotorGearTeeth = 57; //Number of teeth in turret motor gear
    public static double turretGearTeeth = 186; //Number of teeth in turret gear
    public static double turretTicksPerDegree = (turretMotorTPR * turretGearTeeth) / (turretMotorGearTeeth * 360.0);
    public static double turretLimitCCW = 360; // in degrees (No more limits)
    public static double turretLimitCW = -360; // in degrees (No more limits)
    public static double turretSpeedAuto = 0.67; // 0 to 1

    //Camera Tuning Vars
    public static int cameraResolutionWidth = 1280;
    public static int cameraResolutionHeight = 800;
    public static int redTagID = 24;
    public static int blueTagID = 20;
    //Mirror variables for blue side autonomous
    public static int mirrorXCoordinate = -1; //Flip X coordinates
    //Y coordinates remain the same
    public static int mirrorHeading = -1; // in degrees
    public static int artifactHeadingRed = 0; // in degrees
    public static int artifactHeadingBlue = 180; // in degrees
    public static String red = "red";
    public static String blue = "blue";
    public static Vector2d parkPositionLong = new Vector2d(36, -48);
    public static Vector2d shootingPositionLong = new Vector2d(10, -48.75);
    public static Vector2d collectHumanArtifact1 = new Vector2d(60,-60);
    public static Vector2d collectHumanArtifactIdle = new Vector2d(56,-60);
    public static Vector2d collectHumanArtifact2 = new Vector2d(60,-64.75);
    public static Vector2d beginCollectingArtifacts1 = new Vector2d(30, -36);
    public static Vector2d endCollectingArtifacts1 = new Vector2d(59, -36);
    public static Vector2d beginCollectingArtifacts2 = new Vector2d(30, -12);
    public static Vector2d endCollectingArtifacts2 = new Vector2d(59, -12);
    public static Vector2d intermediateStoppingPoint = new Vector2d(48, -12);
    public static Vector2d beginCollectingArtifacts3 = new Vector2d(30, 12);
    public static Vector2d endCollectingArtifacts3 = new Vector2d(51, 12);
    public static Vector2d shootingPositionShort = new Vector2d(18, 18);
    public static Vector2d parkPositionShort = new Vector2d(50, 0);
    public static Vector2d intermediatePressingLever = new Vector2d(48, -12);
    public static Vector2d pressLever = parkPositionShort;
}
