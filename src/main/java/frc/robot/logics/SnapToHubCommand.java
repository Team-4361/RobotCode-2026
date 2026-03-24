package frc.robot.logics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

import java.util.Optional;

public class SnapToHubCommand extends Command {

    // ── Tuning ────────────────────────────────────────────────────────────────

    /** Target distance from hub centre (metres). */
    private static final double HUB_KEEP_DISTANCE_M  = 3.3688;

    /** P-gain for radial snap (m/s per metre of error). */
    private static final double HUB_SNAP_GAIN        = 4.0;

    /** Max speed toward/away from hub (m/s). */
    private static final double HUB_SNAP_MAX_SPEED   = 2.5;

    /** P-gain for heading (rad/s per radian of error). */
    private static final double HEADING_P            = 5.0;

    /** Max angular speed from heading controller (rad/s). */
    private static final double HEADING_MAX_RAD_S    = 4.0;

    /**
     * Tolerance to consider "done":
     *   distance error < DISTANCE_TOLERANCE_M  AND
     *   heading error  < HEADING_TOLERANCE_RAD
     */
    private static final double DISTANCE_TOLERANCE_M   = 0.05; // 5 cm
    private static final double HEADING_TOLERANCE_RAD  = Math.toRadians(3); // 3°

    // ─────────────────────────────────────────────────────────────────────────

    public SnapToHubCommand() {
        // Require drivebase so this blocks any concurrent PathPlanner command
        addRequirements(RobotContainer.drivebase);
    }

    @Override
    public void initialize() {
        // Nothing needed — we calculate fresh every execute() from live pose
    }

    @Override
    public void execute() {
        Pose2d pose       = RobotContainer.drivebase.getPose();
        Translation2d robotPos = pose.getTranslation();

        // Pick alliance hub
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        Translation2d hub = isRed
                ? RobotContainer.HUB_CENTER_RED
                : RobotContainer.HUB_CENTER_BLUE;

        // Hub→robot vector
        Translation2d toRobot  = robotPos.minus(hub);
        double        distance = toRobot.getNorm();
        if (distance < 0.01) return;

        Translation2d unitAway = toRobot.div(distance);

        // ── Radial snap ───────────────────────────────────────────────────
        // distanceError > 0 → too far  → move toward hub (subtract unitAway)
        // distanceError < 0 → too close → move away    (add unitAway)
        double distanceError = distance - HUB_KEEP_DISTANCE_M;
        double snapSpeed = MathUtil.clamp(
                HUB_SNAP_GAIN * distanceError,
                -HUB_SNAP_MAX_SPEED,
                 HUB_SNAP_MAX_SPEED
        );
        double xV = -snapSpeed * unitAway.getX();
        double yV = -snapSpeed * unitAway.getY();

        // ── Heading snap — face the hub ────────────────────────────────────
        Rotation2d desiredHeading = new Rotation2d(
                Math.atan2(-unitAway.getY(), -unitAway.getX()) + Math.PI /2
        );
        double headingError = MathUtil.angleModulus(
                desiredHeading.minus(pose.getRotation()).getRadians()
        );
        double rV = MathUtil.clamp(
                HEADING_P * headingError,
                -HEADING_MAX_RAD_S,
                 HEADING_MAX_RAD_S
        );

        RobotContainer.drivebase.drive(xV, yV, rV, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot cleanly when done (or interrupted by e.g. disable)
        RobotContainer.drivebase.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose       = RobotContainer.drivebase.getPose();
        Translation2d robotPos = pose.getTranslation();

        //Determines the alliance
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        Translation2d hub = isRed
                ? RobotContainer.HUB_CENTER_RED
                : RobotContainer.HUB_CENTER_BLUE;

        Translation2d toRobot  = robotPos.minus(hub);
        double        distance = toRobot.getNorm();
        Translation2d unitAway = distance > 0.01 ? toRobot.div(distance) : new Translation2d(1, 0);

        double distanceError = Math.abs(distance - HUB_KEEP_DISTANCE_M);

        Rotation2d desiredHeading = new Rotation2d(
                Math.atan2(-unitAway.getY(), -unitAway.getX()) + Math.PI /2
        );
        double headingError = Math.abs(MathUtil.angleModulus(
                desiredHeading.minus(pose.getRotation()).getRadians()
        ));

        return distanceError < DISTANCE_TOLERANCE_M
            && headingError  < HEADING_TOLERANCE_RAD;
    }
}