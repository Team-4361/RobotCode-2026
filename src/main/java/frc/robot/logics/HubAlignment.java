package frc.robot.logics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;

import java.util.Optional;

/**
 * Shared math for "face the hub + hold a set distance from it".
 *
 * Both the teleop hub-orbit overlay (teleopController) and the autonomous
 * SnapToHubCommand used to independently re-derive this same geometry
 * (hub vector, distance, desired heading, heading error). Centralizing it
 * here means:
 *   - it's computed once per call instead of being duplicated across files
 *   - SnapToHubCommand can cache one Snapshot per tick and reuse it in
 *     isFinished() instead of re-reading pose + redoing atan2/angleModulus
 *   - tuning/behavior changes only need to happen in one place
 */
public final class HubAlignment {

    private HubAlignment() {}

    /** Immutable result of one alignment computation for a single loop tick. */
    public static final class Snapshot {
        /** Unit vector pointing FROM the hub TOWARD the robot. */
        public final Translation2d unitAway;
        /** Current distance from hub center (m). */
        public final double distance;
        /** distance - keepDistance. Positive = too far, negative = too close. */
        public final double distanceError;
        /** Heading that points the robot's front at the hub. */
        public final Rotation2d desiredHeading;
        /** Wrapped heading error (rad), current heading -> desiredHeading. */
        public final double headingError;

        Snapshot(Translation2d unitAway, double distance, double distanceError,
                 Rotation2d desiredHeading, double headingError) {
            this.unitAway = unitAway;
            this.distance = distance;
            this.distanceError = distanceError;
            this.desiredHeading = desiredHeading;
            this.headingError = headingError;
        }
    }

    /** True if current alliance is Red (defaults to Blue if not yet reported). */
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static Translation2d allianceHub(boolean isRed) {
        return isRed ? RobotContainer.HUB_CENTER_RED : RobotContainer.HUB_CENTER_BLUE;
    }

    /**
     * Computes the hub-relative geometry in one pass.
     * Returns null if the robot is essentially on top of the hub
     * (degenerate — nothing sane to command).
     */
    public static Snapshot compute(Pose2d pose, Translation2d hub, double keepDistanceM) {
        Translation2d toRobot = pose.getTranslation().minus(hub);
        double distance = toRobot.getNorm();
        if (distance < 0.01) return null;

        Translation2d unitAway = toRobot.div(distance);
        Rotation2d desiredHeading = new Rotation2d(Math.atan2(-unitAway.getY(), -unitAway.getX()));
        double headingError = MathUtil.angleModulus(desiredHeading.minus(pose.getRotation()).getRadians());
        double distanceError = distance - keepDistanceM;

        return new Snapshot(unitAway, distance, distanceError, desiredHeading, headingError);
    }

    /** Radial (toward/away from hub) correction speed, positive = away from hub. */
    public static double radialSnapSpeed(double distanceError, double gain, double maxSpeed) {
        return MathUtil.clamp(gain * distanceError, -maxSpeed, maxSpeed);
    }

    /** Rotational correction speed to close headingError. */
    public static double headingSnapSpeed(double headingError, double gain, double maxSpeed) {
        return MathUtil.clamp(gain * headingError, -maxSpeed, maxSpeed);
    }
}