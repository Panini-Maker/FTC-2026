package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.5)
            .forwardZeroPowerAcceleration(-50.1370482674598)
            .lateralZeroPowerAcceleration(-79.46213396002643)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0.0, 0.003, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.06, 0.00015, 0.0012, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0.0, 0.001, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.6, 0.0005, 0.0, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.001, 0.001, 0.2, 0.055))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0.0, 0.001, 0.0, 0.0))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .centripetalScaling(0.0016)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(77.79643621970351)
            .yVelocity(62.88722385196235)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(73)
            .strafePodX(-125)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}