package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.Vector2d;

public class TuningVars {
    public static int odoXOffset = 75; // in mm
    public static int odoYOffset = -146; // in mm
    public static int shootingSlowDownSpeed = 200;
    public static int shotgun = 1900; //Was 2000
    public static int sniper = 2250;
    public static int shootDurationMs = 2400; // in milliseconds
    public static int rampUpTime = 3000; // in milliseconds
    public static int idealVoltage = 12; // in volts

    //Camera Tuning Vars
    public static int cameraResolutionWidth = 1280;
    public static int cameraResolutionHeight = 800;
    public static int redTagID = 24;
    public static int blueTagID = 20;
    //Mirror variables for blue side autonomous
    public static int mirrorXCoordinate = -1; //Flip X coordinates
    //Y coordinates remain the same
    public static int mirrorHeading = -1; // in degrees
    public static String Red = "red";
    public static String Blue = "blue";
    public static Vector2d parkPositionLong = new Vector2d(24, -48);
    public static Vector2d shootingPositionLong = new Vector2d(12, -60);
    public static Vector2d beginCollectingArtifacts1 = new Vector2d(33, -36);
    public static Vector2d endCollectingArtifacts1 = new Vector2d(63, -36);
    public static Vector2d beginCollectingArtifacts2 = new Vector2d(33, -12);
    public static Vector2d endCollectingArtifacts2 = new Vector2d(63, -12);
    public static Vector2d intermediateStoppingPoint = new Vector2d(33, -12);
    public static Vector2d beginCollectingArtifacts3 = new Vector2d(33, 12);
    public static Vector2d endCollectingArtifacts3 = new Vector2d(57, 12);
    public static Vector2d shootingPositionShort = new Vector2d(24, 24);
    public static Vector2d parkPositionShort = new Vector2d(58, 0);
    public static Vector2d intermediatePressingLever = new Vector2d(48, -12);
    public static Vector2d pressLever = parkPositionShort;

}
