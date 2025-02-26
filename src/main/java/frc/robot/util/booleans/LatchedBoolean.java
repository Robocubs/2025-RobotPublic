package frc.robot.util.booleans;

public class LatchedBoolean {
    private final boolean latchedValue;
    private boolean latched = false;
    private boolean value = false;

    public LatchedBoolean(boolean latchedValue) {
        this(latchedValue, !latchedValue);
    }

    public LatchedBoolean(boolean latchedValue, boolean initialValue) {
        this.latchedValue = latchedValue;
        value = !latchedValue;
    }

    public boolean update(boolean value) {
        if (value == latchedValue) {
            this.value = latchedValue;
            latched = true;
        }

        if (latched) {
            return latchedValue;
        }

        this.value = value;
        return value;
    }

    public boolean get() {
        return value;
    }

    public void resetLatch() {
        latched = false;
    }
}
