package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

/**
 * Full ballistics calculator for the FUEL ball shooter.
 *
 * <p><b>Physics models included:</b>
 * <ul>
 *   <li>Fixed 35.1° launch angle (matches the physical hood)</li>
 *   <li>Quadratic aerodynamic drag  (Cd ≈ 0.47 for a foam sphere)</li>
 *   <li>Magnus effect — flywheel backspin generates upward lift, extending
 *       range and improving arc shape</li>
 *   <li>Robot velocity compensation — the ball inherits the robot's
 *       field-relative velocity at the moment of launch; we binary-search
 *       for the extra flywheel contribution needed on top of that</li>
 *   <li>Hub position derived from the robot's {@link Pose2d} via odometry</li>
 * </ul>
 *
 * <p><b>Coordinate convention (for the internal ODE):</b>
 * <ul>
 *   <li>+X  forward along the shot direction (horizontal, toward hub)</li>
 *   <li>+Z  upward</li>
 *   <li>+Y  field-left  (only used for the lateral drift sanity check)</li>
 * </ul>
 *
 * <p><b>Constants to tune on the real robot</b> (in order of importance):
 * <ol>
 *   <li>{@link #EFFICIENCY}    — shoot statically from a known distance,
 *       adjust until the ball lands in the hub.</li>
 *   <li>{@link #SHOOTER_HEIGHT_M} — measure from carpet to ball-CG at launch.</li>
 *   <li>{@link #SPIN_RATIO}    — film in slow-motion; count ball rotations per
 *       wheel rotation.</li>
 *   <li>{@link #CL}            — tweak if trajectory curve doesn't match
 *       simulation at long range.</li>
 * </ol>
 *
 * <p><b>Alliance / field geometry (2022 Rapid React):</b><br>
 * The hub is at the field center: (field_length/2, field_width/2) in the
 * WPILib blue-origin coordinate system.  Both alliances shoot into the same hub.
 */
public class ShooterCalculator {



    /** Field length along the X axis (blue wall → red wall), in meters. */
    public static final double FIELD_LENGTH_M = Units.inchesToMeters(651.22);

    /** Field width along the Y axis (side wall → side wall), in meters. */
    public static final double FIELD_WIDTH_M  = Units.inchesToMeters(317.69);

    /**
     * Blue hub center in the WPILib blue-origin coordinate system.
     *
     * <p>Derived from field measurements:
     * <ul>
     *   <li>X = 182.11 in from the blue alliance wall  = 4.6256 m ≈ 4.630 m</li>
     *   <li>Y = 158.84 in from the nearest side wall   = 4.0345 m ≈ 4.035 m</li>
     * </ul>
     */
    public static final Translation2d HUB_CENTER_BLUE =
            new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

    /**
     * Red hub center in the WPILib blue-origin coordinate system.
     *
     * <p>Derived from field measurements:
     * <ul>
     *   <li>X = 651.22 − 182.11 = 469.11 in from the blue wall = 11.9153 m ≈ 11.915 m</li>
     *   <li>Y = 158.84 in from the nearest side wall            =  4.0345 m ≈  4.035 m</li>
     * </ul>
     */
    public static final Translation2d HUB_CENTER_RED =
            new Translation2d(FIELD_LENGTH_M - Units.inchesToMeters(182.11),
                              Units.inchesToMeters(158.84));

    /** Height of the hub opening's front lip above carpet (m) — 72 in. */
    public static final double HUB_LIP_HEIGHT_M   = Units.inchesToMeters(72.0);

    /**
     * Extra vertical clearance added above the lip so the ball safely clears
     * the rim.  Increase if you're seeing rim-outs on the near edge.
     */
    public static final double HUB_CLEARANCE_M    = Units.inchesToMeters(3.0);

    /** Effective apex-height the trajectory must reach (m). */
    public static final double TARGET_HEIGHT_M    = HUB_LIP_HEIGHT_M + HUB_CLEARANCE_M;

    //  Ball properties

    /** Diameter  (m) — 5.91 in. */
    public static final double BALL_DIAMETER_M  = Units.inchesToMeters(5.91);
    public static final double BALL_RADIUS_M    = BALL_DIAMETER_M / 2.0;

    /** Cross-sectional area (m²). */
    public static final double BALL_AREA_M2     = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M;

    /** Mass (kg) — midpoint of 0.448–0.500 lbs. */
    public static final double BALL_MASS_KG     = Units.lbsToKilograms(0.474);

    //  Aerodynamics

    /** Air density at sea level (kg/m³).  Adjust for venue altitude if needed. */
    public static final double AIR_DENSITY      = 1.225;

    /**
     * Drag coefficient (Cd) for a rough foam sphere.
     * Smooth sphere ≈ 0.47; rougher surface ≈ 0.50–0.55.
     * Set to 0.0 to disable drag (useful for comparing with pure kinematics).
     */
    public static final double CD               = 0.47;

    /**
     * Magnus lift coefficient (CL).
     * Relates the spin parameter (ω·r / v) to a dimensionless transverse force.
     * For foam balls at FRC exit speeds: empirically 0.18 – 0.28.
     * Positive CL with backspin → upward Magnus force → ball carries farther
     * with a flatter arc, allowing a slightly lower exit speed.
     * Set to 0.0 to disable Magnus effect entirely.
     */
    public static final double CL               = 0.22;

    //  Shooter / robot geometry  — TUNE THESE

    /** Fixed hood angle above horizontal (°). */
    public static final double LAUNCH_ANGLE_DEG = 35.1;
    public static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);

    /**
     * Height of ball center above carpet at the instant it leaves the shooter (m).
     * Measure on the physical robot.  36–42 in is typical.
     */
    public static final double SHOOTER_HEIGHT_M = Units.inchesToMeters(38.0);

    /**
     * Flywheel-to-ball surface-speed transfer efficiency (0–1).
     * A compressed foam ball receives roughly 80–92% of the wheel's surface speed.
     * PRIMARY TUNING CONSTANT — adjust first when doing static shot tuning.
     */
    public static final double EFFICIENCY       = 0.85;

    /**
     * Shooter wheel radius (m) — 4 in diameter wheel → 2 in radius.
     */
    public static final double WHEEL_RADIUS_M   = Units.inchesToMeters(1.0);

    /**
     * Ball angular-velocity ratio: ball_spin_rps / wheel_spin_rps.
     * For a single-wheel top shooter in direct contact with the ball:
     *   ball_omega ≈ (wheel_surface_speed × contact_efficiency) / ball_radius
     * This constant collapses that relationship into a single tunable scalar.
     * Positive → backspin (top-mounted flywheel) → upward Magnus force.
     * Film shots in slow-motion to verify.
     */
    public static final double SPIN_RATIO       = 0.90;

    /** Maximum allowable flywheel command (matches ShooterSubsystem soft limit). */
    public static final double MAX_RPM          = 6000.0;

    //  Numerical integration settings

    private static final double DT              = 0.001;  // RK4 time step (s)
    private static final double MAX_FLIGHT_TIME = 3.0;    // abort integration after this (s)
    private static final double G               = 9.80665;

    //  Public API

    /**
     * Calculate the required flywheel RPM to score into the hub.
     *
     * <p>Accounts for robot velocity, fixed 35.1° launch angle, aerodynamic drag,
     * and Magnus lift from backspin.
     *
     * <p>Typical usage in a Command:
     * <pre>
     *   return run(() -> {
     *       AngularVelocity rpm = ShooterCalculator.getRPM(
     *           swerve.getPose(),
     *           swerve.getFieldVelocity(),
     *           isRedAlliance()
     *       );
     *       shooter.setVelocity(rpm);
     *   });
     * </pre>
     *
     * @param robotPose   Current robot {@link Pose2d} from swerve odometry.
     * @param robotSpeeds Current robot <em>field-relative</em> {@link ChassisSpeeds}.
     *                    Obtain via {@code swerve.getFieldVelocity()}.
     * @param redAlliance {@code true} if the robot is on the red alliance.
     * @return Required flywheel {@link AngularVelocity}, clamped to {@link #MAX_RPM}.
     *         Returns 0 RPM if no valid solution exists.
     */
    public static AngularVelocity getRPM(Pose2d robotPose,
                                          ChassisSpeeds robotSpeeds,
                                          boolean redAlliance) {
        Translation2d hubCenter = redAlliance ? HUB_CENTER_RED : HUB_CENTER_BLUE;
        double dist2d  = robotPose.getTranslation().getDistance(hubCenter);

        // Direction unit-vector from robot toward hub in the field XY plane
        double dx  = hubCenter.getX() - robotPose.getX();
        double dy  = hubCenter.getY() - robotPose.getY();
        if (dist2d < 0.05) return RPM.of(0); // standing on top of hub

        double ux  = dx / dist2d;
        double uy  = dy / dist2d;

        // Decompose robot velocity into:
        //   vForward — component toward the hub (+ve = moving toward hub)
        //   vLateral — component perpendicular to the shot direction
        double vForward = robotSpeeds.vxMetersPerSecond * ux
                        + robotSpeeds.vyMetersPerSecond * uy;
        double vLateral = -robotSpeeds.vxMetersPerSecond * uy
                        +  robotSpeeds.vyMetersPerSecond * ux;

        double rpm = binarySearchRPM(dist2d, vForward, vLateral);
        if (Double.isNaN(rpm) || rpm <= 0) return RPM.of(0);
        return RPM.of(Math.min(rpm, MAX_RPM));
    }

    /**
     * Overload that assumes the robot is stationary.
     * Handy for testing or pre-match spin-up.
     *
     * @param robotPose   Current robot {@link Pose2d}.
     * @param redAlliance Alliance color.
     * @return Required flywheel {@link AngularVelocity}.
     */
    public static AngularVelocity getRPM(Pose2d robotPose, boolean redAlliance) {
        return getRPM(robotPose, new ChassisSpeeds(), redAlliance);
    }

    /**
     * Returns the 2-D horizontal distance from the robot to its alliance hub (m).
     *
     * @param robotPose   Robot's current pose.
     * @param redAlliance True if on the red alliance.
     * @return Distance in meters.
     */
    public static double getDistanceToHubMeters(Pose2d robotPose, boolean redAlliance) {
        Translation2d hub = redAlliance ? HUB_CENTER_RED : HUB_CENTER_BLUE;
        return robotPose.getTranslation().getDistance(hub);
    }

    //  Binary search over exit speed

    /**
     * Binary-search over the ball's flywheel-contributed exit speed (m/s) to
     * find the minimum value that causes the simulated trajectory to clear
     * {@link #TARGET_HEIGHT_M} at the hub distance.
     *
     * @param dist2d   Horizontal distance to hub (m).
     * @param vForward Robot velocity toward hub (m/s).
     * @param vLateral Robot velocity perpendicular to shot (m/s).
     * @return Flywheel RPM, or NaN on failure.
     */
    private static double binarySearchRPM(double dist2d,
                                           double vForward,
                                           double vLateral) {
        double lo = 0.5;
        double hi = 30.0;

        // Check if even the maximum speed can reach the target
        if (!simulationReachesTarget(hi, dist2d, vForward)) {
            // Fallback to drag-free kinematics (better than returning 0)
            return rawKinematicsRPM(dist2d, vForward);
        }

        for (int i = 0; i < 60; i++) {
            double mid = (lo + hi) * 0.5;
            if (simulationReachesTarget(mid, dist2d, vForward)) {
                hi = mid;
            } else {
                lo = mid;
            }
            if (hi - lo < 0.01) break;
        }

        return exitSpeedToRPM((lo + hi) * 0.5);
    }

    //  RK4 trajectory simulation

    /**
     * Simulate the ball trajectory and return whether it reaches
     * {@link #TARGET_HEIGHT_M} at the hub's horizontal distance.
     *
     * <p><b>Forces modelled per time-step:</b>
     * <pre>
     *   Drag:   F_d = -1/2p Cd A |v|^2  (opposing velocity)
     *   Magnus: F_m = +1/2p CL A |v| (w(omega)·r/|v|)  (+Z for backspin moving in +X)
     *   Gravity: F_g = −m·g  (−Z)
     * </pre>
     *
     * The spin angular velocity decays slightly over the flight (gyroscopic
     * precession and bearing friction), but for FRC-length shots the effect is
     * small enough to treat ω as constant.
     *
     * @param exitSpeed Flywheel-contributed exit speed (m/s).
     * @param dist2d    Horizontal distance to hub (m).
     * @param vForward  Robot velocity component toward hub (m/s).
     * @return {@code true} if the simulated ball clears the target height.
     */
    private static boolean simulationReachesTarget(double exitSpeed,
                                                     double dist2d,
                                                     double vForward) {
        // Initial conditions
        double x  = 0.0;
        double z  = SHOOTER_HEIGHT_M;
        double vx = exitSpeed * Math.cos(LAUNCH_ANGLE_RAD) + vForward;
        double vz = exitSpeed * Math.sin(LAUNCH_ANGLE_RAD);

        // Ball backspin angular velocity (rad/s)
        // ω_ball = (wheel_surface_speed × SPIN_RATIO) / BALL_RADIUS_M
        // wheel surface speed ≈ exitSpeed / EFFICIENCY
        double omegaBall = (exitSpeed / EFFICIENCY) * SPIN_RATIO / BALL_RADIUS_M;

        // Pre-compute drag/Magnus scale factors
        double kDrag   = (0.5 * AIR_DENSITY * CD * BALL_AREA_M2) / BALL_MASS_KG;
        double kMagnus = (0.5 * AIR_DENSITY * CL * BALL_AREA_M2) / BALL_MASS_KG;

        for (double t = 0; t < MAX_FLIGHT_TIME; t += DT) {
            // ── RK4 step ────────────────────────────────────────────────────
            double[] s  = {vx, vz, x, z};
            double[] k1 = accel(s, kDrag, kMagnus, omegaBall);

            double[] s2 = {vx + 0.5*DT*k1[0], vz + 0.5*DT*k1[1],
                            x  + 0.5*DT*k1[2],  z  + 0.5*DT*k1[3]};
            double[] k2 = accel(s2, kDrag, kMagnus, omegaBall);

            double[] s3 = {vx + 0.5*DT*k2[0], vz + 0.5*DT*k2[1],
                            x  + 0.5*DT*k2[2],  z  + 0.5*DT*k2[3]};
            double[] k3 = accel(s3, kDrag, kMagnus, omegaBall);

            double[] s4 = {vx + DT*k3[0], vz + DT*k3[1],
                            x  + DT*k3[2],  z  + DT*k3[3]};
            double[] k4 = accel(s4, kDrag, kMagnus, omegaBall);

            vx += (DT / 6.0) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
            vz += (DT / 6.0) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
            x  += (DT / 6.0) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]);
            z  += (DT / 6.0) * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]);

            // ── Check if we have reached the hub horizontal distance ─────────
            if (x >= dist2d - BALL_RADIUS_M) {
                return z >= TARGET_HEIGHT_M;
            }

            // Bail early if the ball has already hit the floor
            if (z < 0) return false;
        }
        return false;
    }

    /**
     * Compute state derivatives [dvx/dt, dvz/dt, dx/dt, dz/dt] for RK4.
     *
     * <p>State vector s = [vx, vz, x, z].
     *
     * <p>Drag opposes the velocity vector.
     * Magnus (backspin on a +X moving ball) adds +Z (upward) acceleration.
     *
     * @param s          State [vx, vz, x, z].
     * @param kDrag      Pre-computed drag scale (1/2pCdA/m).
     * @param kMagnus    Pre-computed Magnus scale (1/2pClA/m).
     * @param omegaBall  Ball spin angular velocity (rad/s, backspin positive).
     * @return Derivatives [dvx/dt, dvz/dt, dx/dt, dz/dt].
     */
    private static double[] accel(double[] s,
                                    double kDrag,
                                    double kMagnus,
                                    double omegaBall) {
        double vx    = s[0];
        double vz    = s[1];
        double speed = Math.hypot(vx, vz);

        // Drag (opposes velocity)
        double ax = -kDrag * speed * vx;
        double az = -kDrag * speed * vz;

        // Magnus (backspin creates upward force for +X motion)
        // Spin parameter Sp = ω·r / |v|
        // Magnus acceleration magnitude = kMagnus × |v| × Sp  (in +Z for backspin)
        if (speed > 0.01) {
            double sp      = (omegaBall * BALL_RADIUS_M) / speed;
            double magnus  = kMagnus * speed * sp; // upward (+Z)
            az += magnus;
        }

        // Gravity
        az -= G;

        // Derivatives: d(vx)/dt = ax, d(vz)/dt = az, d(x)/dt = vx, d(z)/dt = vz
        return new double[]{ax, az, vx, vz};
    }

    //  Conversion helpers

    /**
     * Convert ball exit speed (m/s) → flywheel RPM.
     *
     * @param exitSpeedMs Ball exit speed at the wheel contact point (m/s).
     * @return Flywheel RPM.
     */
    public static double exitSpeedToRPM(double exitSpeedMs) {
        double surfaceSpeed = exitSpeedMs / EFFICIENCY;
        double omega        = surfaceSpeed / WHEEL_RADIUS_M;
        return omega * 60.0 / (2.0 * Math.PI);
    }

    /**
     * Convert flywheel RPM → ball exit speed (m/s).
     *
     * @param rpm Flywheel RPM.
     * @return Ball exit speed in m/s.
     */
    public static double rpmToExitSpeed(double rpm) {
        double omega        = rpm * 2.0 * Math.PI / 60.0;
        double surfaceSpeed = omega * WHEEL_RADIUS_M;
        return surfaceSpeed * EFFICIENCY;
    }

    //  Drag-free fallback

    /**
     * Analytical no-drag, no-Magnus solution used as a fallback when the
     * simulation cannot converge.
     *
     * <p>Derivation: set y(x = dist) = HUB_LIP_HEIGHT and solve for v₀.
     *
     * @param dist2d   Horizontal distance (m).
     * @param vForward Robot forward velocity (m/s) — subtracted from required v₀.
     * @return Flywheel RPM.
     */
    private static double rawKinematicsRPM(double dist2d, double vForward) {
        double tanA   = Math.tan(LAUNCH_ANGLE_RAD);
        double cosA   = Math.cos(LAUNCH_ANGLE_RAD);
        double deltaY = HUB_LIP_HEIGHT_M - SHOOTER_HEIGHT_M;
        double denom  = dist2d * tanA - deltaY;
        if (denom <= 0) return MAX_RPM;
        double v0 = Math.sqrt((G * dist2d * dist2d) / (2.0 * cosA * cosA * denom));
        v0 = Math.max(v0 - vForward, 0.1);
        return exitSpeedToRPM(v0);
    }

    //  Debug utilities

    /**
     * Print a static distance-to-RPM lookup table to stdout.
     * Call this from disabled periodic or a test harness to sanity-check values.
     *
     * <p>Example: {@code ShooterCalculator.printLookupTable(36, 240, 12);}
     *
     * @param minInches  Starting distance (in).
     * @param maxInches  Ending distance (in).
     * @param stepInches Step size (in).
     */
    public static void printLookupTable(double minInches,
                                         double maxInches,
                                         double stepInches) {
        System.out.printf("%-22s %-22s %-22s%n",
                          "Distance (in)", "Exit Speed (m/s)", "RPM (static, no drag)");
        System.out.println("─".repeat(68));
        for (double d = minInches; d <= maxInches; d += stepInches) {
            double dm        = Units.inchesToMeters(d);
            double rpm       = binarySearchRPM(dm, 0.0, 0.0);
            double exitSpeed = rpmToExitSpeed(Math.min(rpm, MAX_RPM));
            System.out.printf("%-22.1f %-22.3f %-22.1f%n",
                              d, exitSpeed, Math.min(rpm, MAX_RPM));
        }
    }
}