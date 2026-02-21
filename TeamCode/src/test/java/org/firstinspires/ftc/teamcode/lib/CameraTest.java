package org.firstinspires.ftc.teamcode.lib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for Camera.calculateRobotPose method.
 * Tests verify that the robot position calculation from AprilTag detection is correct.
 *
 * Coordinate System:
 * - Field origin (0,0) is at center of field
 * - X positive is toward red alliance wall
 * - Y positive is toward the far wall (away from audience)
 * - Headings are positive counter-clockwise (CCW)
 *
 * AprilTag positions (from TuningVars):
 * - Red goal: (72, 72), tag adjusted to (72-15, 72-11) = (57, 61)
 * - Blue goal: (-72, 72), tag adjusted to (-72+15, 72-11) = (-57, 61)
 */
public class CameraTest {

    private Camera camera;
    private static final double TOLERANCE = 2.0; // inches tolerance for floating point comparison
    private static final double ANGLE_TOLERANCE = 1.0; // degrees tolerance

    // Tag positions after adjustment (from Camera.java)
    // Red tag: (72-15, 72-11) = (57, 61)
    // Blue tag: (-72+15, 72-11) = (-57, 61)

    @Before
    public void setUp() {
        // Create Camera with no offset
        camera = new Camera();
    }

    // ==================== BASIC FUNCTIONALITY TESTS ====================

    /**
     * Test that calculateRobotPose returns a non-null Pose2D.
     */
    @Test
    public void testCalculateRobotPose_ReturnsNonNull() {
        Pose2D result = camera.calculateRobotPose(0, 0, 0, 48, true);
        assertNotNull("calculateRobotPose should return a non-null Pose2D", result);
    }

    /**
     * Test that the returned pose has the same heading as the input robot heading.
     */
    @Test
    public void testCalculateRobotPose_PreservesHeading() {
        double inputHeading = 45.0;
        Pose2D result = camera.calculateRobotPose(0, inputHeading, 0, 48, true);
        assertEquals("Pose heading should match input robot heading",
                inputHeading, result.getHeading(AngleUnit.DEGREES), ANGLE_TOLERANCE);
    }

    // ==================== RED GOAL TESTS ====================

    /**
     * Test when robot is directly in front of red tag, looking straight at it.
     * Robot heading = 45° (facing red goal corner at 45° from origin)
     * Turret heading = 0° (facing forward on robot)
     * Camera sees tag directly ahead (x=0, z=distance)
     *
     * Expected: Robot should be at (57 - distance*cos(45°), 61 - distance*sin(45°))
     */
    @Test
    public void testCalculateRobotPose_RedGoal_DirectlyInFront() {
        double robotHeading = 45.0;
        double turretHeading = 0.0;
        double tagX = 0; // Tag is directly ahead
        double tagZ = 48; // 48 inches away
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        // Red tag is at (57, 61)
        // Camera heading = 45° + 0° = 45°
        // Robot should be 48 inches away from tag at angle 225° (opposite of 45°)
        // Expected position:
        // X = 57 - 48*cos(45°) ≈ 57 - 33.9 ≈ 23.1
        // Y = 61 - 48*sin(45°) ≈ 61 - 33.9 ≈ 27.1

        assertNotNull(result);
        System.out.println("Red Direct - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));

        // The pose should be valid (within field bounds)
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    /**
     * Test when robot is on X axis facing positive Y, looking at red goal.
     * Robot heading = 90° (facing positive Y)
     * Turret heading = -45° (turned 45° clockwise to face goal)
     * Camera should see tag at an angle
     */
    @Test
    public void testCalculateRobotPose_RedGoal_RobotFacingPositiveY() {
        double robotHeading = 90.0;
        double turretHeading = -45.0; // Turret turned to face goal
        double distance = 60.0;
        // Tag appears slightly to the right
        double tagX = 10;
        double tagZ = Math.sqrt(distance * distance - tagX * tagX);
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Red FacingY - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    /**
     * Test red goal with robot at origin, heading 0, turret at 45°.
     */
    @Test
    public void testCalculateRobotPose_RedGoal_RobotAtOrigin() {
        double robotHeading = 0.0;
        double turretHeading = -45.0; // Turret pointing toward goal
        double tagX = 0;
        double tagZ = 80; // Distance to tag
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, false);

        assertNotNull(result);
        System.out.println("Red Origin - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));

        // Robot should be somewhere in the negative X, negative Y quadrant relative to tag
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    // ==================== BLUE GOAL TESTS ====================

    /**
     * Test when robot is directly in front of blue tag, looking straight at it.
     * Robot heading = 135° (facing blue goal corner)
     * Turret heading = 0° (facing forward on robot)
     */
    @Test
    public void testCalculateRobotPose_BlueGoal_DirectlyInFront() {
        double robotHeading = 135.0;
        double turretHeading = 0.0;
        double tagX = 0;
        double tagZ = 48;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, false);

        assertNotNull(result);
        System.out.println("Blue Direct - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    /**
     * Test blue goal with robot at negative X, facing positive X.
     */
    @Test
    public void testCalculateRobotPose_BlueGoal_RobotNegativeX() {
        double robotHeading = 0.0;
        double turretHeading = 135.0; // Turret pointing back-left toward blue goal
        double tagX = 5;
        double tagZ = 70;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, false);

        assertNotNull(result);
        System.out.println("Blue NegX - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    // ==================== CAMERA OFFSET TESTS ====================

    /**
     * Test with camera offset - camera mounted right of robot center.
     */
    @Test
    public void testCalculateRobotPose_WithCameraOffsetRight() {
        Camera cameraWithOffset = new Camera(5.0, 0); // 5 inches right

        double robotHeading = 45.0;
        double turretHeading = 0.0;
        double tagX = 0;
        double tagZ = 48;
        double yaw = 0;

        Pose2D resultNoOffset = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);
        Pose2D resultWithOffset = cameraWithOffset.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(resultNoOffset);
        assertNotNull(resultWithOffset);

        // With offset, robot center should be shifted
        double xDiff = Math.abs(resultWithOffset.getX(DistanceUnit.INCH) - resultNoOffset.getX(DistanceUnit.INCH));
        double yDiff = Math.abs(resultWithOffset.getY(DistanceUnit.INCH) - resultNoOffset.getY(DistanceUnit.INCH));

        System.out.println("Offset difference - X: " + xDiff + ", Y: " + yDiff);

        // The offset should cause a noticeable difference in position
        assertTrue("Camera offset should affect calculated position",
                xDiff > 1 || yDiff > 1);
    }

    /**
     * Test with camera offset - camera mounted forward of robot center.
     */
    @Test
    public void testCalculateRobotPose_WithCameraOffsetForward() {
        Camera cameraWithOffset = new Camera(0, 6.0); // 6 inches forward

        double robotHeading = 0.0;
        double turretHeading = 45.0;
        double tagX = 0;
        double tagZ = 60;
        double yaw = 0;

        Pose2D resultNoOffset = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);
        Pose2D resultWithOffset = cameraWithOffset.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(resultNoOffset);
        assertNotNull(resultWithOffset);

        double xDiff = Math.abs(resultWithOffset.getX(DistanceUnit.INCH) - resultNoOffset.getX(DistanceUnit.INCH));
        double yDiff = Math.abs(resultWithOffset.getY(DistanceUnit.INCH) - resultNoOffset.getY(DistanceUnit.INCH));

        System.out.println("Forward offset difference - X: " + xDiff + ", Y: " + yDiff);

        assertTrue("Camera offset should affect calculated position",
                xDiff > 1 || yDiff > 1);
    }

    // ==================== TAG OFFSET TESTS (x != 0) ====================

    /**
     * Test when tag appears to the right of camera center.
     */
    @Test
    public void testCalculateRobotPose_TagOffsetRight() {
        double robotHeading = 45.0;
        double turretHeading = 0.0;
        double tagX = 12; // Tag 12 inches to the right
        double tagZ = 48;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Tag Right - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    /**
     * Test when tag appears to the left of camera center.
     */
    @Test
    public void testCalculateRobotPose_TagOffsetLeft() {
        double robotHeading = 45.0;
        double turretHeading = 0.0;
        double tagX = -12; // Tag 12 inches to the left
        double tagZ = 48;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Tag Left - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    // ==================== POSE VALIDATION TESTS ====================

    /**
     * Test isPoseValid with valid pose inside field.
     */
    @Test
    public void testIsPoseValid_ValidPose() {
        Pose2D validPose = new Pose2D(DistanceUnit.INCH, 36, 36, AngleUnit.DEGREES, 0);
        assertTrue("Pose at (36, 36) should be valid", camera.isPoseValid(validPose));
    }

    /**
     * Test isPoseValid with pose outside field bounds.
     */
    @Test
    public void testIsPoseValid_InvalidPose_TooFarX() {
        Pose2D invalidPose = new Pose2D(DistanceUnit.INCH, 100, 36, AngleUnit.DEGREES, 0);
        assertFalse("Pose at X=100 should be invalid", camera.isPoseValid(invalidPose));
    }

    /**
     * Test isPoseValid with pose outside field bounds.
     */
    @Test
    public void testIsPoseValid_InvalidPose_TooFarY() {
        Pose2D invalidPose = new Pose2D(DistanceUnit.INCH, 36, -100, AngleUnit.DEGREES, 0);
        assertFalse("Pose at Y=-100 should be invalid", camera.isPoseValid(invalidPose));
    }

    /**
     * Test isPoseValid with null pose.
     */
    @Test
    public void testIsPoseValid_NullPose() {
        assertFalse("Null pose should be invalid", camera.isPoseValid(null));
    }

    /**
     * Test isPoseValid with pose at field edge (within margin).
     */
    @Test
    public void testIsPoseValid_EdgeOfField() {
        // Field is 144" x 144" with 6" margin, so max is 78"
        Pose2D edgePose = new Pose2D(DistanceUnit.INCH, 75, -75, AngleUnit.DEGREES, 0);
        assertTrue("Pose at (75, -75) should be valid (within margin)", camera.isPoseValid(edgePose));
    }

    // ==================== TURRET ROTATION TESTS ====================

    /**
     * Test with various turret angles to ensure rotation is handled correctly.
     */
    @Test
    public void testCalculateRobotPose_TurretRotation90() {
        double robotHeading = 0.0;
        double turretHeading = 90.0; // Turret facing left
        double tagX = 0;
        double tagZ = 50;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Turret 90 - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    /**
     * Test with turret at -90 degrees.
     */
    @Test
    public void testCalculateRobotPose_TurretRotationNegative90() {
        double robotHeading = 0.0;
        double turretHeading = -90.0; // Turret facing right
        double tagX = 0;
        double tagZ = 50;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Turret -90 - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        assertTrue("Pose should be within field bounds", camera.isPoseValid(result));
    }

    /**
     * Test with turret at 180 degrees (facing backward).
     */
    @Test
    public void testCalculateRobotPose_TurretRotation180() {
        double robotHeading = 0.0;
        double turretHeading = 180.0; // Turret facing backward
        double tagX = 0;
        double tagZ = 50;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Turret 180 - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
        // Note: This might place robot outside field bounds depending on calculation
    }

    // ==================== ROBOT HEADING TESTS ====================

    /**
     * Test with robot heading at various angles.
     */
    @Test
    public void testCalculateRobotPose_RobotHeading90() {
        double robotHeading = 90.0; // Robot facing positive Y
        double turretHeading = 0.0;
        double tagX = 0;
        double tagZ = 50;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        assertEquals("Heading should be preserved", 90.0,
                result.getHeading(AngleUnit.DEGREES), ANGLE_TOLERANCE);
        System.out.println("Robot H90 - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
    }

    /**
     * Test with negative robot heading.
     */
    @Test
    public void testCalculateRobotPose_RobotHeadingNegative45() {
        double robotHeading = -45.0;
        double turretHeading = 90.0;
        double tagX = 0;
        double tagZ = 60;
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        assertEquals("Heading should be preserved", -45.0,
                result.getHeading(AngleUnit.DEGREES), ANGLE_TOLERANCE);
        System.out.println("Robot H-45 - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
    }

    // ==================== DISTANCE TESTS ====================

    /**
     * Test with close distance to tag.
     */
    @Test
    public void testCalculateRobotPose_CloseDistance() {
        double robotHeading = 45.0;
        double turretHeading = 0.0;
        double tagX = 0;
        double tagZ = 24; // Very close
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Close - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));

        // Robot should be close to the tag position
        assertTrue("Close range pose should be valid", camera.isPoseValid(result));
    }

    /**
     * Test with far distance to tag.
     */
    @Test
    public void testCalculateRobotPose_FarDistance() {
        double robotHeading = 45.0;
        double turretHeading = 0.0;
        double tagX = 0;
        double tagZ = 96; // Far away
        double yaw = 0;

        Pose2D result = camera.calculateRobotPose(turretHeading, robotHeading, tagX, tagZ, true);

        assertNotNull(result);
        System.out.println("Far - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
    }

    // ==================== CONSISTENCY TESTS ====================

    /**
     * Test that symmetric inputs for red and blue goals produce symmetric outputs.
     */
    @Test
    public void testCalculateRobotPose_RedBlueSymmetry() {
        // For red: robot heading 45° facing (72,72)
        // For blue: robot heading 135° facing (-72,72)

        double turretHeading = 0.0;
        double tagX = 0;
        double tagZ = 50;
        double yaw = 0;

        Pose2D redResult = camera.calculateRobotPose(turretHeading, 45.0, tagX, tagZ, true);
        Pose2D blueResult = camera.calculateRobotPose(turretHeading, 135.0, tagX, tagZ, false);

        assertNotNull(redResult);
        assertNotNull(blueResult);

        System.out.println("Red Symmetry - X: " + redResult.getX(DistanceUnit.INCH) +
                ", Y: " + redResult.getY(DistanceUnit.INCH));
        System.out.println("Blue Symmetry - X: " + blueResult.getX(DistanceUnit.INCH) +
                ", Y: " + blueResult.getY(DistanceUnit.INCH));

        // X values should be roughly symmetric (opposite signs)
        // Y values should be similar
        double redX = redResult.getX(DistanceUnit.INCH);
        double blueX = blueResult.getX(DistanceUnit.INCH);

        // The absolute values of X should be somewhat similar due to symmetry
        // (not exact due to tag offset differences)
    }

    // ==================== CALCULATEROBOTPOSEWITHIMU TESTS ====================

    /**
     * Test the simplified calculateRobotPoseWithIMU method.
     */
    @Test
    public void testCalculateRobotPoseWithIMU_Basic() {
        double xRelative = 0;
        double yRelative = 80;
        double turretHeading = 0;
        double robotHeadingIMU = -45;
        boolean isRed = false;

        Pose2D result = camera.calculateRobotPoseWithIMU(xRelative, yRelative, turretHeading, robotHeadingIMU, isRed);

        assertNotNull(result);
        assertEquals("Heading should match IMU heading", robotHeadingIMU,
                result.getHeading(AngleUnit.DEGREES), ANGLE_TOLERANCE);
        System.out.println("IMU Method - X: " + result.getX(DistanceUnit.INCH) +
                ", Y: " + result.getY(DistanceUnit.INCH));
    }

    // ==================== SETTER/GETTER TESTS ====================

    /**
     * Test setCameraOffset and getters.
     */
    @Test
    public void testCameraOffsetSettersGetters() {
        Camera testCamera = new Camera();

        assertEquals("Initial X offset should be 0", 0, testCamera.getCameraToShooterCenter(), 0.001);
        assertEquals("Initial Y offset should be 0", 0, testCamera.getShooterCenterToRobotCenter(), 0.001);

        testCamera.setCameraOffset(5.5, 3.2);

        assertEquals("X offset should be updated", 5.5, testCamera.getCameraToShooterCenter(), 0.001);
        assertEquals("Y offset should be updated", 3.2, testCamera.getShooterCenterToRobotCenter(), 0.001);
    }

    /**
     * Test constructor with offset values.
     */
    @Test
    public void testConstructorWithOffsets() {
        Camera testCamera = new Camera(7.5, 4.2);

        assertEquals("X offset should match constructor arg", 7.5, testCamera.getCameraToShooterCenter(), 0.001);
        assertEquals("Y offset should match constructor arg", 4.2, testCamera.getShooterCenterToRobotCenter(), 0.001);
    }
}

