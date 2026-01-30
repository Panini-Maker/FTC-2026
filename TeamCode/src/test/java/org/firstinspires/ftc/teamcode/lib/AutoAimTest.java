package org.firstinspires.ftc.teamcode.lib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for AutoAim.calculateTargetAngle method.
 * Tests verify that the turret angle calculation is correct for various robot positions and headings.
 */
public class AutoAimTest {

    private AutoAim autoAim;

    // Test constants matching TuningVars defaults
    private static final double TURRET_PHYSICAL_OFFSET = 180.0;
    private static final double TURRET_LIMIT_CCW = 180.0;
    private static final double TURRET_LIMIT_CW = -135.0;
    private static final double TOLERANCE = 1.0; // degrees tolerance for floating point comparison
    private static final double POWER_TOLERANCE = 0.5; // Check if it is from 1 to 0

    @Before
    public void setUp() {
        // Create AutoAim with null turret and telemetry since we're only testing calculations
        // isRed = true means targeting (72, 72)
        autoAim = new AutoAim(null, null, true);
    }

    /**
     * Test when robot is at origin (0,0) facing positive X axis (heading = 0).
     * Red goal is at (72, 72), so angle from robot to goal is 45 degrees.
     * With robot heading 0 and turretPhysicalOffset = 180:
     * turretTargetAngle = 45 - 0 + 180 = 225, normalized to -135 (within limits)
     */
    @Test
    public void testCalculateTargetAngle_RobotAtOrigin_FacingPositiveX() {
        double robotX = 0;
        double robotY = 0;
        double robotHeading = 0; // Facing positive X
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Absolute angle to (72,72) from (0,0) is 45 degrees
        // Relative to robot front = 45 - 0 = 45
        // Turret target = 45 + 180 = 225, normalized to -135
        assertEquals(-135.0, targetAngle, TOLERANCE);
        assertEquals(-0.5, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        System.out.println(autoAim.calculatePID(targetAngle,currentTurretAngle));
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test when robot is at origin facing the goal directly (heading = 45 degrees).
     * Turret should be at turretPhysicalOffset (180) to point forward.
     */
    @Test
    public void testCalculateTargetAngle_RobotFacingGoalDirectly() {
        double robotX = 0;
        double robotY = 0;
        double robotHeading = 45; // Facing directly at red goal (72, 72)
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Absolute angle to goal = 45
        // Relative to robot front = 45 - 45 = 0
        // Turret target = 0 + 180 = 180
        assertEquals(180.0, targetAngle, TOLERANCE);
        assertEquals(0.5, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        System.out.println(autoAim.calculatePID(targetAngle, currentTurretAngle));
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test when robot is at origin facing negative X (heading = 180 degrees).
     */
    @Test
    public void testCalculateTargetAngle_RobotFacingNegativeX() {
        double robotX = 0;
        double robotY = 0;
        double robotHeading = 180; // Facing negative X
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Absolute angle to goal = 45
        // Relative to robot front = 45 - 180 = -135
        // Turret target = -135 + 180 = 45
        assertEquals(45.0, targetAngle, TOLERANCE);
        assertEquals(0.5, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        System.out.println(autoAim.calculatePID(targetAngle, currentTurretAngle));
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test when robot is closer to the goal.
     */
    @Test
    public void testCalculateTargetAngle_RobotCloserToGoal() {
        double robotX = 36;
        double robotY = 36;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Robot at (36, 36), goal at (72, 72)
        // dx = 36, dy = 36
        // Absolute angle = atan2(36, 36) = 45 degrees
        // Relative = 45 - 0 = 45
        // Turret target = 45 + 180 = 225, normalized to -135
        assertEquals(-135.0, targetAngle, TOLERANCE);
        assertEquals(-0.8, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        System.out.println(autoAim.calculatePID(targetAngle, currentTurretAngle));
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test blue team targeting (goal at -72, 72).
     */
    @Test
    public void testCalculateTargetAngle_BlueTeam() {
        // Create AutoAim for blue team
        AutoAim blueAutoAim = new AutoAim(null, null, false);

        double robotX = 0;
        double robotY = 0;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = blueAutoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Blue goal at (-72, 72)
        // Absolute angle = atan2(72, -72) = 135 degrees
        // Relative = 135 - 0 = 135
        // Turret target = 135 + 180 = 315, normalized to -45
        assertEquals(-45.0, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test that turret angle is clamped to CCW limit.
     */
    @Test
    public void testCalculateTargetAngle_ClampedToCCWLimit() {
        // Position robot such that target angle would exceed CCW limit
        double robotX = 0;
        double robotY = 0;
        double robotHeading = 45; // Facing goal directly, turret should be at 180
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Turret limit CCW is 180, so 180 should be allowed
        assertTrue(targetAngle <= TURRET_LIMIT_CCW);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test that turret angle is clamped to CW limit.
     */
    @Test
    public void testCalculateTargetAngle_ClampedToCWLimit() {
        double robotX = 0;
        double robotY = 0;
        double robotHeading = 0; // Facing positive X
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Turret limit CW is -135
        assertTrue(targetAngle >= TURRET_LIMIT_CW);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test with negative robot heading.
     */
    @Test
    public void testCalculateTargetAngle_NegativeHeading() {
        double robotX = 0;
        double robotY = 0;
        double robotHeading = -45; // Facing between positive X and negative Y
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Absolute angle to goal = 45
        // Relative = 45 - (-45) = 90
        // Turret target = 90 + 180 = 270, normalized to -90
        assertEquals(-90.0, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test robot positioned on the same Y line as the goal.
     */
    @Test
    public void testCalculateTargetAngle_SameYAsGoal() {
        double robotX = 0;
        double robotY = 72;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Goal at (72, 72), robot at (0, 72)
        // dx = 72, dy = 0
        // Absolute angle = atan2(0, 72) = 0 degrees
        // Relative = 0 - 0 = 0
        // Turret target = 0 + 180 = 180
        assertEquals(180.0, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test robot positioned on Y-axis.
     */
    @Test
    public void testCalculateTargetAngleY_Axis_1() {
        double robotX = 0;
        double robotY = -48;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Goal at (72, 72), robot at (0, 72)
        // dx = 72, dy = 0
        // Absolute angle = atan2(0, 72) = 0 degrees
        // Relative = 0 - 0 = 0
        // Turret target = 0 + 180 = 180
        assertEquals(-120.9637, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test robot positioned on Y-axis.
     */
    @Test
    public void testCalculateTargetAngleY_Axis_2() {
        double robotX = 0;
        double robotY = -24;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Goal at (72, 72), robot at (0, 72)
        // dx = 72, dy = 0
        // Absolute angle = atan2(0, 72) = 0 degrees
        // Relative = 0 - 0 = 0
        // Turret target = 0 + 180 = 180
        assertEquals(-126.86989, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test robot positioned on Y-axis.
     */
    @Test
    public void testCalculateTargetAngleY_Axis_3() {
        double robotX = 0;
        double robotY = 24;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Goal at (72, 72), robot at (0, 72)
        // dx = 72, dy = 0
        // Absolute angle = atan2(0, 72) = 0 degrees
        // Relative = 0 - 0 = 0
        // Turret target = 0 + 180 = 180
        // Limited to -135 due to hardware, supposed to be -146.3099
        assertEquals(-135, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test robot positioned on Y-axis.
     */
    @Test
    public void testCalculateTargetAngleY_Axis_4() {
        double robotX = 0;
        double robotY = 48;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Goal at (72, 72), robot at (0, 72)
        // dx = 72, dy = 0
        // Absolute angle = atan2(0, 72) = 0 degrees
        // Relative = 0 - 0 = 0
        // Turret target = 0 + 180 = 180
        //Limited to -135 due to hardware, supposed to be -161.565
        assertEquals(-135, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), 0.0001);
    }

    /**
     * Test robot PID calculation.
     */
    @Test
    public void testCalculatePID_1() {
        double robotX = 0;
        double robotY = -24;
        double robotHeading = 0;
        double currentTurretAngle = 0; // Facing behind robot

        double targetAngle = autoAim.calculateTargetAngle(robotX, robotY, robotHeading);

        // Goal at (72, 72), robot at (0, 72)
        // dx = 72, dy = 0
        // Absolute angle = atan2(0, 72) = 0 degrees
        // Relative = 0 - 0 = 0
        // Turret target = 0 + 180 = 180
        assertEquals(-126.86989, targetAngle, TOLERANCE);
        assertEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle), POWER_TOLERANCE);
        assertNotEquals(0, autoAim.calculatePID(targetAngle,currentTurretAngle),0.0001);
    }

    /**
     * Test smaller error
     */
    @Test
    public void testSmallerPIDError() {
        double currentTurretAngle = -130; // Facing behind robot
        assertEquals(-0.8, autoAim.calculatePID(-135,currentTurretAngle), POWER_TOLERANCE);
        System.out.println(autoAim.calculatePID(-135, currentTurretAngle));
    }
}