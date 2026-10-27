package frc.robot.logics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Distance -> RPM / time-of-flight tables plus the shoot-on-the-move solver.
 *
 * Shoot-on-the-move: the fuel inherits the robot's velocity, so we aim at a
 * "virtual target" = hub - robotVelocity * tof. The tables (built from
 * stationary shots) are then looked up with the distance to that virtual target.
 * tof depends on distance and distance depends on tof, so we iterate.
 *
 * Coordinates: same field frame as drivebase.getPose() / getFieldVelocity().
 */
public final class ShotCalculator {

    private ShotCalculator() {}

    // =========================================================================
    // SHOT TABLE  — !!! PLACEHOLDER VALUES, REPLACE WITH MEASURED DATA !!!
    // Rows MUST be sorted by ascending distance.
    // {distance (m, shooter exit -> hub centre), shooter RPM, time of flight (s)}
    // =========================================================================
    private static final double[][] TABLE = {
        {1.949754, 2500, 5},
        {2.289754, 3000, 5.16},
        {2.949754, 3350, 5.97},
    };

    private static final InterpolatingDoubleTreeMap RPM_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

    static {
        for (double[] row : TABLE) {
            RPM_MAP.put(row[0], row[1]);
            TOF_MAP.put(row[0], row[2]);
        }
    }

    /** Smallest / largest distance we have data for. We never look up outside this. */
    public static final double MIN_DISTANCE_M = TABLE[0][0];
    public static final double MAX_DISTANCE_M = TABLE[TABLE.length - 1][0];

    // =========================================================================
    // Tuning
    // =========================================================================

    /** Stay this far inside the table's min/max so we never touch the edges. */
    public static final double RANGE_MARGIN_M = 0.15;

    /** Max radial (toward/away from hub) speed the range-keeping logic may command. */
    public static final double MAX_RADIAL_SPEED_MPS = 2.0;

    /** Fixed-point iterations for the virtual target (3-5 is plenty). */
    private static final int ITERATIONS = 5;

    /** Floor on tof used when converting a distance error to a velocity bound. */
    private static final double MIN_TOF_FOR_BOUND_S = 0.2;

    /** Shooter exit point relative to the odometry centre (robot frame, +x fwd, +y left). */
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(
        Constants.ShooterConstants.SHOOTER_FORWARD_OFFSET_M,
        Constants.ShooterConstants.SHOOTER_LEFT_OFFSET_M);

    // =========================================================================
    // Result
    // =========================================================================

    public static final class Solution {
        /** Point the robot should actually aim at (hub shifted against robot velocity). */
        public final Translation2d virtualTarget;
        /** Field heading that points the robot's front at the virtual target. */
        public final Rotation2d aimHeading;
        /** Wrapped heading error (rad): aimHeading - current heading. */
        public final double headingErrorRad;
        /** Rate the aim line to the hub is rotating (rad/s) — use as feedforward. */
        public final double headingRateFF;
        /** Shooter -> virtual target distance (m). NOT clamped. */
        public final double effectiveDistanceM;
        /** Shooter -> real hub distance (m). */
        public final double realDistanceM;
        /** Time of flight used (s). */
        public final double tofSeconds;
        /** Shooter RPM from the table (looked up at the clamped effective distance). */
        public final double rpm;
        /** True if the effective distance is inside the table's data range. */
        public final boolean inRange;
        /** Robot translational speed (m/s). */
        public final double speedMps;
        /** Unit vector pointing from the hub toward the shooter. */
        public final Translation2d unitAway;

        Solution(Translation2d virtualTarget, Rotation2d aimHeading, double headingErrorRad,
                 double headingRateFF, double effectiveDistanceM, double realDistanceM,
                 double tofSeconds, double rpm, boolean inRange, double speedMps,
                 Translation2d unitAway) {
            this.virtualTarget = virtualTarget;
            this.aimHeading = aimHeading;
            this.headingErrorRad = headingErrorRad;
            this.headingRateFF = headingRateFF;
            this.effectiveDistanceM = effectiveDistanceM;
            this.realDistanceM = realDistanceM;
            this.tofSeconds = tofSeconds;
            this.rpm = rpm;
            this.inRange = inRange;
            this.speedMps = speedMps;
            this.unitAway = unitAway;
        }

        /**
         * Limits a radial velocity (+ = away from hub) so that, over one time of
         * flight, the effective distance stays inside
         * [MIN + margin, MAX - margin]. If we are already outside that band the
         * bounds force a velocity back toward it, so this also acts as the
         * "bring the driver back into range" controller.
         */
        public double clampRadialVelocity(double vAway) {
            double t = Math.max(tofSeconds, MIN_TOF_FOR_BOUND_S);
            double lo = (MIN_DISTANCE_M + RANGE_MARGIN_M - realDistanceM) / t;
            double hi = (MAX_DISTANCE_M - RANGE_MARGIN_M - realDistanceM) / t;
            lo = MathUtil.clamp(lo, -MAX_RADIAL_SPEED_MPS, MAX_RADIAL_SPEED_MPS);
            hi = MathUtil.clamp(hi, -MAX_RADIAL_SPEED_MPS, MAX_RADIAL_SPEED_MPS);
            return MathUtil.clamp(vAway, lo, hi);
        }

        /** True if the real distance is inside the margin band (with some slack, m). */
        public boolean isRealDistanceInsideMargin(double slackM) {
            return realDistanceM >= MIN_DISTANCE_M + RANGE_MARGIN_M - slackM
                && realDistanceM <= MAX_DISTANCE_M - RANGE_MARGIN_M + slackM;
        }
    }

    // =========================================================================
    // Solver
    // =========================================================================

    public static double clampDistance(double distanceM) {
        return MathUtil.clamp(distanceM, MIN_DISTANCE_M, MAX_DISTANCE_M);
    }

    public static Solution solve(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d hub) {
        Translation2d shooterPos = robotPose.getTranslation()
            .plus(SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
        Translation2d vel = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        double realDistance = shooterPos.getDistance(hub);

        // Fixed-point iteration for tof / virtual target.
        double tof = TOF_MAP.get(clampDistance(realDistance));
        Translation2d virtualTarget = hub;
        double effDistance = realDistance;
        for (int i = 0; i < ITERATIONS; i++) {
            virtualTarget = hub.minus(vel.times(tof));
            effDistance = shooterPos.getDistance(virtualTarget);
            tof = TOF_MAP.get(clampDistance(effDistance));
        }

        Translation2d toVirtual = virtualTarget.minus(shooterPos);
        Rotation2d aimHeading = toVirtual.getNorm() > 1e-6 ? toVirtual.getAngle() : robotPose.getRotation();
        double headingError = MathUtil.angleModulus(
            aimHeading.minus(robotPose.getRotation()).getRadians());

        // d(bearing to hub)/dt for a moving robot: (a x (-v)) / |a|^2, a = hub - shooter.
        Translation2d a = hub.minus(shooterPos);
        double denom = Math.max(a.getSquaredNorm(), 1e-6);
        double headingRateFF = (-a.getX() * vel.getY() + a.getY() * vel.getX()) / denom;

        Translation2d unitAway = realDistance > 1e-3
            ? shooterPos.minus(hub).div(realDistance)
            : new Translation2d(1.0, 0.0);

        boolean inRange = effDistance >= MIN_DISTANCE_M && effDistance <= MAX_DISTANCE_M;
        double rpm = RPM_MAP.get(clampDistance(effDistance));

        return new Solution(virtualTarget, aimHeading, headingError, headingRateFF,
                            effDistance, realDistance, tof, rpm, inRange, vel.getNorm(), unitAway);
    }

    /** Convenience: solve using the live drivebase pose/velocity and alliance hub. */
    public static Solution solveNow() {
        return solve(
            RobotContainer.drivebase.getPose(),
            RobotContainer.drivebase.getFieldVelocity(),
            HubAlignment.allianceHub(HubAlignment.isRedAlliance()));
    }
}