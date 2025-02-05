package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

public class PhoenixUtil {
    private static final int defaultMaxAttempts = 5;

    public static void tryUntilOk(Supplier<StatusCode> command) {
        tryUntilOk(defaultMaxAttempts, command);
    }

    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            if (command.get().isOK()) {
                break;
            }
        }
    }
}
