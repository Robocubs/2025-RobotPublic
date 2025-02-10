package frc.robot.util.booleans;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Time;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/*
 * A boolean value that is latched for a minimum amount of time.
 * The latchedValue is the value that will not change until the minimum time has passed.
 */
public class TimeLatchedBoolean {
    private final double minimumTime;
    private final boolean latchedValue;
    private final DoubleSupplier timestamp;

    private double lastLatchedValueTimestamp;
    private boolean value;

    public TimeLatchedBoolean(Time minimumTime, boolean latchedValue) {
        this(minimumTime, latchedValue, Logger::getTimestamp);
    }

    public TimeLatchedBoolean(Time minimumTime, boolean latchedValue, DoubleSupplier timestamp) {
        this.minimumTime = minimumTime.in(Seconds);
        this.latchedValue = latchedValue;
        this.timestamp = timestamp == null ? Logger::getTimestamp : timestamp;
        lastLatchedValueTimestamp = this.timestamp.getAsDouble() - this.minimumTime;
        value = !latchedValue;
    }

    public boolean update(boolean newValue) {
        if (newValue == this.latchedValue) {
            lastLatchedValueTimestamp = timestamp.getAsDouble();
            value = newValue;
            return value;
        }

        this.value = timestamp.getAsDouble() - this.lastLatchedValueTimestamp >= minimumTime ? newValue : !newValue;
        return this.value;
    }
}
