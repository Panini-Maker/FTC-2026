package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
@Config
public class TuningVars {
    //Odometry Tuning Vars
    public static int odoXOffset = 73; // in mm
    public static int odoYOffset = -125; // in mm
    //Auto End Position - Updated at end of autonomous, used by TeleOp
    public static double autoEndX = 0; // in inches
    public static double autoEndY = 0; // in inches
    public static double autoEndHeading = 0; // in degrees
    public static double autoEndTurretHeading = 0; // in degrees
    public static double timeToShoot = 0.1; // Seconds

    // Method to save robot position at end of autonomous
    public static void saveEndPosition(double x, double y, double heading, double turretHeading) {
        autoEndX = x;
        autoEndY = y;
        autoEndHeading = heading;
        autoEndTurretHeading = turretHeading;
    }
    //Shooter Tuning Vars
    public static double shooterKp = 0.005;//0.25
    public static double shooterKi = 0.0011;//0.0002
    public static double shooterKd = 0;//0.002
    public static int shootingSlowDownSpeed = 200;
    public static int shotgun = 1550; //Was 1600
    public static int shotgunTeleOp = 1470;
    public static int sniper = 2000; //Was 1550
    public static int sniperAuto = 1900; // Was 2100 but shooter cannot reach that speed reliably, Changed to 1950
    // Was 1950, but not enough for autonomous shots
    // Shots too high at 1976 rpm, 1925 causes problem where it hits backboard sometimes
    public static int shooterIdle = 0; // Run when not shooting during teleop
    public static int idle = 800; // Was 1200
    public static int shootingToleranceAuto = 50; // in RPM
    public static int shootingToleranceTeleOp = 50; // in RPM
    public static int shootDurationMs = 2500; // in milliseconds
    public static int rampUpTime = 3000; // in milliseconds
    public static int idealVoltage = 12; // in volts
    public static double turretMotorTPR = 537.7; // ticks per revolution
    public static double turretMotorGearTeeth = 57; //Number of teeth in turret motor gear
    public static double turretGearTeeth = 186; //Number of teeth in turret gear
    public static double turretTicksPerDegree = (turretMotorTPR * turretGearTeeth) / (turretMotorGearTeeth * 360.0);
    public static double turretLimitCCW = 175; // in degrees (Limits added back in)
    public static double turretLimitCW = -130; // in degrees (Limits added back in)
    public static double turretSpeedAuto = 0.8; // 0 to 1

    // Turret PID Tuning Vars for Auto Aim
    public static double turretKp = 0.04; // Proportional constant
    public static double turretKi = 0.0; // Integral constant
    public static double turretKd = 0.0; // Derivative constant
    public static double turretMinPower = 0.0; // Minimum power to overcome friction
    public static double turretMaxPower = 0.8; // Maximum power for turret
    public static double turretTolerance = 2.0; // Tolerance in degrees
    public static double turretPhysicalOffset = 180.0; // Turret encoder 0 faces back of robot (180° from front)

    // Auto Aim Target Positions (center of field is origin 0,0)
    public static Vector2d redGoalPosition = new Vector2d(72, 72); // Red goal corner
    public static Vector2d blueGoalPosition = new Vector2d(-72, 72); // Blue goal corner
    public static boolean targetIsRed = true;

    // Odometry heading convention adjustment
    // Set to -1 if odometry uses CW positive (opposite of atan2's CCW positive)
    // Set to 1 if odometry uses CCW positive (same as atan2)
    public static double odometryHeadingSign = 1; // GoBilda Pinpoint typically uses CW positive
    //Nope, CCW is positive

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
    public static Vector2d parkPositionShort = new Vector2d(45, 0);
    public static Vector2d intermediatePressingLever = new Vector2d(48, -12);
    public static Vector2d pressLever = parkPositionShort;
    public static Vector2d odoResetPosRed = new Vector2d(-65.25, -65.25);
}
