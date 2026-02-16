package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.blueGoalPosition;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.redGoalPosition;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Camera class for AprilTag-based relocalization.
 *
 * Calculates robot field position from AprilTag detection data.
 *
 * Coordinate System:
 * - Field origin (0,0) is at center of field
 * - X positive is toward red alliance wall
 * - Y positive is toward the far wall (away from audience)
 * - Headings are positive counter-clockwise (CCW)
 *
 * AprilTag Detection Values:
 * - yaw: Rotation of the tag relative to camera (positive CCW)
 * - x: Horizontal offset of tag from camera center (positive right)
 * - z: Distance from camera to tag (always positive, forward)
 *
 * Turret and Robot Headings:
 * - Turret heading: Angle of turret relative to robot (0 = forward, positive CCW)
 * - Robot heading: Angle of robot on field (0 = facing +X, positive CCW)
 */
public class Camera {

    // Camera mounting offset from robot center (in inches)
    // These should be tuned for your specific robot
    private double cameraToShooterCenter = 0;  // Positive = camera is to the right of center
    private double shooterCenterToRobotCenter = 0;  // Positive = camera is forward of center

    /**
     * Default constructor with no camera offset.
     */
    public Camera() {
        this.cameraToShooterCenter = 0;
        this.shooterCenterToRobotCenter = 0;
    }

    /**
     * Constructor with camera mounting offsets.
     *
     * @param cameraToShooterCenter Distance from robot center to camera (positive = right)
     * @param shooterCenterToRobotCenter Distance from robot center to camera (positive = forward)
     */
    public Camera(double cameraToShooterCenter, double shooterCenterToRobotCenter) {
        this.cameraToShooterCenter = cameraToShooterCenter;
        this.shooterCenterToRobotCenter = shooterCenterToRobotCenter;
    }

    /**
     * Calculates robot field pose from AprilTag detection.
     *
     * FORMULA DERIVATION:
     *
     * 1. Camera sees tag at position (x_rel, z_rel) in camera frame
     *    - x_rel: horizontal offset (positive = tag is to the right)
     *    - z_rel: forward distance (always positive)
     *
     * 2. Yaw is the angle of the tag face relative to camera
     *    - yaw = 0 means tag is facing directly at camera
     *    - positive yaw = tag is rotated CCW from camera's view
     *
     * 3. Camera heading on field = robot_heading + turret_heading
     *    (assuming turret 0 = facing forward on robot)
     *
     * 4. To find camera position on field:
     *    - We know the tag position (goal corner)
     *    - We know how far and at what angle the camera sees the tag
     *    - Camera position = tag position - (vector from camera to tag in field frame)
     *
     * 5. Convert camera-relative (x_rel, z_rel) to field frame:
     *    - Angle from camera to tag in camera frame: atan2(x_rel, z_rel)
     *    - Distance: sqrt(x_rel² + z_rel²)
     *    - Angle in field frame: camera_heading + angle_to_tag
     *
     * 6. Robot position = camera position - camera offset (rotated by robot heading)
     *
     * @param yawDegrees AprilTag yaw in degrees (rotation of tag face, positive CCW)
     * @param turretHeadingDegrees Turret heading relative to robot in degrees (positive CCW)
     * @param robotHeadingDegrees Robot heading on field in degrees (positive CCW)
     * @param xRelativeInches Horizontal offset of tag from camera in inches (positive = right)
     * @param yRelativeInches Forward distance to tag in inches (positive = forward)
     * @param isRedGoal True if detecting red goal AprilTag, false for blue
     * @return Calculated robot pose on field, or null if calculation fails
     */
    public Pose2D calculateRobotPose(
            double yawDegrees,
            double turretHeadingDegrees,
            double robotHeadingDegrees,
            double xRelativeInches,
            double yRelativeInches,
            boolean isRedGoal) {

        // ==================== STEP 1: Get goal position ====================
        double tagX, tagY;
        if (isRedGoal) {
            tagX = redGoalPosition.x - 15; // Adjust for tag being 15 inches in front of goal corner
            tagY = redGoalPosition.y - 11; // Adjust for tag being 11 inches to the right of goal corner
        } else {
            tagX = blueGoalPosition.x + 15; // Adjust for tag being 15 inches in front of goal corner
            tagY = blueGoalPosition.y - 11; // Adjust for tag being 11 inches to the right of goal corner
        }

        // ==================== STEP 2: Calculate camera heading on field ====================
        // Camera heading = robot heading + turret heading
        // Both are positive CCW, so we add them
        double cameraHeadingDegrees = robotHeadingDegrees + turretHeadingDegrees;
        double cameraHeadingRadians = -Math.toRadians(cameraHeadingDegrees);

        // ==================== STEP 3: Calculate angle and distance from camera to tag ====================
        // In camera frame: x_rel is horizontal (positive right), z_rel is forward
        // Angle from camera forward direction to tag (positive CCW)

        // Distance from camera to tag
        double distanceToTag = Math.sqrt(xRelativeInches * xRelativeInches + yRelativeInches * yRelativeInches);

        // ==================== STEP 4: Calculate angle to tag in field frame ====================
        // Angle from camera to tag in field frame
        double angleToTagFieldFrame = Math.PI - (cameraHeadingRadians + Math.atan(xRelativeInches / yRelativeInches));

        // ==================== STEP 5: Calculate camera heading in field ====================
        double alpha = Math.toRadians(robotHeadingDegrees + turretHeadingDegrees);
        double beta = Math.toRadians(robotHeadingDegrees);

        // ==================== STEP 6: Calculate robot position on field ====================
        double robotX, robotY;

        robotX = tagX - distanceToTag * Math.cos(angleToTagFieldFrame) + cameraToShooterCenter * Math.cos(alpha) + shooterCenterToRobotCenter * Math.cos(beta);
        robotY = tagY - distanceToTag * Math.sin(angleToTagFieldFrame) - cameraToShooterCenter * Math.sin(alpha) + shooterCenterToRobotCenter * Math.sin(beta);

        // ==================== STEP 7: Return pose ====================
        return new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.DEGREES, robotHeadingDegrees);
    }

    /**
     * Simplified version that uses IMU for robot heading.
     * Call this when you trust the IMU heading and just need X, Y from AprilTag.
     *
     * @param xRelativeInches Horizontal offset of tag from camera in inches
     * @param zRelativeInches Forward distance to tag in inches
     * @param turretHeadingDegrees Current turret heading in degrees
     * @param robotHeadingFromIMU Robot heading from IMU in degrees (positive CCW)
     * @param isRedGoal True if red goal, false if blue
     * @return Robot pose on field
     */
    public Pose2D calculateRobotPoseWithIMU(
            double xRelativeInches,
            double zRelativeInches,
            double turretHeadingDegrees,
            double robotHeadingFromIMU,
            boolean isRedGoal) {

        // Use 0 for yaw since we're trusting IMU for heading
        return calculateRobotPose(0, turretHeadingDegrees, robotHeadingFromIMU,
                                   xRelativeInches, zRelativeInches, isRedGoal);
    }

    /**
     * Sets the camera mounting offset from robot center.
     *
     * @param xOffset Distance right from robot center (positive = right)
     * @param yOffset Distance forward from robot center (positive = forward)
     */
    public void setCameraOffset(double xOffset, double yOffset) {
        this.cameraToShooterCenter = xOffset;
        this.shooterCenterToRobotCenter = yOffset;
    }

    /**
     * Gets the current camera X offset.
     */
    public double getCameraToShooterCenter() {
        return cameraToShooterCenter;
    }

    /**
     * Gets the current camera Y offset.
     */
    public double getShooterCenterToRobotCenter() {
        return shooterCenterToRobotCenter;
    }

    /**
     * Validates that the calculated pose is within field bounds.
     *
     * @param pose The pose to validate
     * @return True if pose is within field bounds, false otherwise
     */
    public boolean isPoseValid(Pose2D pose) {
        if (pose == null) return false;

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);

        // Field is roughly 144" x 144" centered at origin
        // Allow some margin for measurement error
        double fieldHalfSize = 72 + 6; // 6 inch margin

        return Math.abs(x) <= fieldHalfSize && Math.abs(y) <= fieldHalfSize;
    }
}


