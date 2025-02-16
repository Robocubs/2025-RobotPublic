package frc.robot.util.booleans;

import lombok.AllArgsConstructor;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
@AllArgsConstructor
public class LatchedBoolean {
    private final boolean latchedValue;
    private boolean latched = false;

    public boolean update(boolean value) {
        if (latched) {
            return latchedValue;
        }

        if (value == latchedValue) {
            latched = true;
        }

        return value;
    }

    public boolean get() {
        return latchedValue;
    }

    public void resetLatch() {
        latched = false;
    }
}
