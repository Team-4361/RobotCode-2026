package frc.robot.logics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.Optional;

public final class HubAlignment {

    private HubAlignment() {}

    // Cached from Constants so we only do the trig setup once.
    private static final double SHOOTER_OFFSET_MAG_M = Math.hypot(
        Constants.ShooterConstants.SHOOTER_FORWARD_OFFSET_M,
        Constants.ShooterConstants.SHOOTER_LEFT_OFFSET_M);
    private static final double SHOOTER_OFFSET_ANGLE_RAD = Math.atan2(
        Constants.ShooterConstants.SHOOTER_LEFT_OFFSET_M,
        Constants.ShooterConstants.SHOOTER_FORWARD_OFFSET_M);

    /** Immutable result of one alignment computation for a single loop tick. */
    public static final class Snapshot {
        /** Unit vector pointing FROM the hub TOWARD the robot's odometry center. */
        public final Translation2d unitAway;
        /** True shooter-exit-to-hub distance (m) — accounts for mount offset. */
        public final double distance;
        /** distance - keepDistance. Positive = too far, negative = too close. */
        public final double distanceError;
        /** Heading that points the SHOOTER (not just the chassis) at the hub. */
        public final Rotation2d desiredHeading;
        /** Wrapped heading error (rad), current heading -> desiredHeading. */
        public final double headingError;
        /** Raw odometry-center-to-hub distance, for telemetry/reference. */
        public final double centerDistance;

        Snapshot(Translation2d unitAway, double distance, double distanceError,
                 Rotation2d desiredHeading, double headingError, double centerDistance) {
            this.unitAway = unitAway;
            this.distance = distance;
            this.distanceError = distanceError;
            this.desiredHeading = desiredHeading;
            this.headingError = headingError;
            this.centerDistance = centerDistance;
        }
    }

    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static Translation2d allianceHub(boolean isRed) {
        return isRed ? RobotContainer.HUB_CENTER_RED : RobotContainer.HUB_CENTER_BLUE;
    }

    /**
     * Computes hub-relative geometry, correcting for the shooter's physical
     * offset from the odometry-tracked center (see Constants.ShooterConstants).
     * Returns null if the robot's center is essentially on top of the hub.
     */
    public static Snapshot compute(Pose2d pose, Translation2d hub, double keepDistanceM) {
        Translation2d toRobot = pose.getTranslation().minus(hub);
        double centerDistance = toRobot.getNorm();
        if (centerDistance < 0.01) return null;

        Translation2d unitAway = toRobot.div(centerDistance);

        // Heading that points the CHASSIS CENTER at the hub (old behavior).
        double centerHeadingRad = Math.atan2(-unitAway.getY(), -unitAway.getX());

        // Correct for the shooter's mount offset so the actual shot — not just
        // the centerline — is aimed at the hub. Solves the triangle formed by
        // the odometry center, the shooter exit point, and the hub.
        double beta = 0.0;
        double shooterDistance = centerDistance;
        if (SHOOTER_OFFSET_MAG_M > 1e-6) {
            double asinArg = MathUtil.clamp(
                (SHOOTER_OFFSET_MAG_M * Math.sin(SHOOTER_OFFSET_ANGLE_RAD)) / centerDistance,
                -1.0, 1.0);
            beta = Math.asin(asinArg);
            shooterDistance = centerDistance * Math.cos(beta)
                - SHOOTER_OFFSET_MAG_M * Math.cos(SHOOTER_OFFSET_ANGLE_RAD);
        }

        Rotation2d desiredHeading = new Rotation2d(centerHeadingRad - beta);
        double headingError = MathUtil.angleModulus(desiredHeading.minus(pose.getRotation()).getRadians());
        double distanceError = shooterDistance - keepDistanceM;

        return new Snapshot(unitAway, shooterDistance, distanceError, desiredHeading, headingError, centerDistance);
    }

    public static double radialSnapSpeed(double distanceError, double gain, double maxSpeed) {
        return MathUtil.clamp(gain * distanceError, -maxSpeed, maxSpeed);
    }

    public static double headingSnapSpeed(double headingError, double gain, double maxSpeed) {
        return MathUtil.clamp(gain * headingError, -maxSpeed, maxSpeed);
    }
}