package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggeredAlert {
    private final Alert alert;

    public TriggeredAlert(String message, AlertType type, BooleanSupplier trigger) {
        this(message, type, new Trigger(trigger));
    }

    public TriggeredAlert(String message, AlertType type, Trigger trigger) {
        alert = new Alert(message, type);
        trigger.onTrue(
                Commands.runOnce(() -> alert.set(true)).ignoringDisable(true).withName("EnableAlert"));
        trigger.onFalse(
                Commands.runOnce(() -> alert.set(false)).ignoringDisable(true).withName("DisableAlert"));
        alert.set(trigger.getAsBoolean());
    }
}
