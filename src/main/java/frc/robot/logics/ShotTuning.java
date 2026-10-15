package frc.robot.logics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.TunableNumber;

/**
 * Dashboard-driven shot tuning.
 *
 * HOW TO USE (teleop, enabled):
 *  1. Tick  "ShotTune/Enabled"  on the dashboard.
 *  2. Type a distance into "ShotTune/Target Distance (m)". The robot faces the
 *     hub and drives to that shooter->hub distance (you can still strafe around
 *     the hub; your forward/back input is ignored).
 *  3. Type an RPM into "ShotTune/RPM". The shooter spins to it.
 *  4. Hold the shoot button (joystickR 2) to feed once at speed + aimed.
 *  5. Happy with a shot? Type the measured flight time into
 *     "ShotTune/Time Of Flight (s)" (0 if unknown) and press "ShotTune/Log Point".
 *     A paste-ready row is appended to "ShotTune/Logged Rows" and the console.
 *  6. Untick "ShotTune/Enabled" to go back to normal driving.
 *
 * Tuning ignores the table's min/max, so you can test outside the current range.
 */
public final class ShotTuning {

    private ShotTuning() {}

    private static final String ENABLED_KEY = "ShotTune/Enabled";
    private static final String LOG_KEY     = "ShotTune/Logged Rows";

    private static final TunableNumber targetDistance =
        new TunableNumber("ShotTune/Target Distance (m)", 2.35);
    private static final TunableNumber rpm =
        new TunableNumber("ShotTune/RPM", 3000);
    private static final TunableNumber tof =
        new TunableNumber("ShotTune/Time Of Flight (s)", 0.0);

    private static final double MIN_DISTANCE_M = 0.5;
    private static final double MAX_DISTANCE_M = 8.0;
    private static final double MAX_RPM        = 6000.0;

    private static final StringBuilder rows = new StringBuilder();

    /** Call once at startup so the dashboard entries/buttons appear. */
    public static void init() {
        SmartDashboard.putBoolean(ENABLED_KEY, false);
        SmartDashboard.putString(LOG_KEY, "(none)");
        SmartDashboard.putData("ShotTune/Log Point",
            Commands.runOnce(ShotTuning::logPoint).ignoringDisable(true).withName("Log Point"));
        SmartDashboard.putData("ShotTune/Clear Log",
            Commands.runOnce(ShotTuning::clearLog).ignoringDisable(true).withName("Clear Log"));
    }

    public static boolean isEnabled() {
        return SmartDashboard.getBoolean(ENABLED_KEY, false);
    }

    public static double getDistance() {
        return MathUtil.clamp(targetDistance.get(), MIN_DISTANCE_M, MAX_DISTANCE_M);
    }

    public static double getRpm() {
        return MathUtil.clamp(rpm.get(), 0.0, MAX_RPM);
    }

    /** RPM the shooter should be at right now: tuned value in tune mode, table value otherwise. */
    public static double commandedRpm() {
        return isEnabled() ? getRpm() : ShotCalculator.solveNow().rpm;
    }

    private static void logPoint() {
        // Record the ACTUAL current distance, not the typed one.
        double d = ShotCalculator.solveNow().realDistanceM;
        String row = String.format("{%.2f, %.0f, %.2f},", d, getRpm(), tof.get());
        if (rows.length() > 0) rows.append(" \n");
        rows.append(row);
        SmartDashboard.putString(LOG_KEY, rows.toString());
        System.out.println("[ShotTune] " + row);
        DataLogManager.log("[ShotTune] " + row);
    }

    private static void clearLog() {
        rows.setLength(0);
        SmartDashboard.putString(LOG_KEY, "(none)");
    }
}