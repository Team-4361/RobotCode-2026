package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;

/**
 * A SmartDashboard-backed number that you can tweak live in Shuffleboard/
 * Elastic/AdvantageScope without redeploying code. Call poll() once per
 * periodic() — it only fires the callback when the value actually changes,
 * so it's cheap to call every loop.
 */
public class TunableNumber {
    private final String key;
    private final double defaultValue;
    private double lastValue;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        SmartDashboard.putNumber(key, defaultValue);
        this.lastValue = defaultValue;
    }

    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    /** Runs onChange(newValue) and returns true only if the dashboard value changed since the last poll(). */
    public boolean poll(Consumer<Double> onChange) {
        double current = get();
        if (current != lastValue) {
            lastValue = current;
            onChange.accept(current);
            return true;
        }
        return false;
    }
}