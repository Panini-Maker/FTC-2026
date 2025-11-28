package org.firstinspires.ftc.teamcode.lib;

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
    public static int heading = 180; // in degrees
}
