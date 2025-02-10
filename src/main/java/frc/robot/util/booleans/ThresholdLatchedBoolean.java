package frc.robot.util.booleans;

import lombok.AllArgsConstructor;
import lombok.RequiredArgsConstructor;

/*
 * A boolean that must pass a threshold to flip.
 */
@RequiredArgsConstructor
@AllArgsConstructor
public class ThresholdLatchedBoolean {
    private final double minThreshold;
    private final double maxThreshold;
    private final boolean highValue;

    private boolean value;

    public static ThresholdLatchedBoolean fromThresholdTolerance(
            double threshold, double tolerance, boolean highValue) {
        return new ThresholdLatchedBoolean(threshold - tolerance, threshold + tolerance, highValue);
    }

    public static ThresholdLatchedBoolean fromThresholdTolerance(
            double threshold, double tolerance, boolean highValue, boolean initialValue) {
        return new ThresholdLatchedBoolean(threshold - tolerance, threshold + tolerance, highValue, initialValue);
    }

    public boolean update(double input) {
        if (input > maxThreshold) {
            value = highValue;
        } else if (input < minThreshold) {
            value = !highValue;
        }

        return value;
    }
}
