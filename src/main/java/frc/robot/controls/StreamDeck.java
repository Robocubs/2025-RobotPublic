package frc.robot.controls;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class StreamDeck extends SubsystemBase {
    private final Map<StreamDeckButton, Button> buttons = new HashMap<>();
    private final Map<StreamDeckButton, Alert> alerts = new HashMap<>();

    private static record Button(LoggedNetworkButton pressed, BooleanSupplier selected, BooleanPublisher activePub) {}

    @Override
    public void periodic() {
        buttons.values().forEach(button -> button.activePub.set(button.selected.getAsBoolean()));
    }

    public StreamDeck configureButtons(Consumer<ButtonConfiguration> config) {
        var configuration = new ButtonConfiguration();
        config.accept(configuration);

        var nt = NetworkTableInstance.getDefault();
        var deckTable = nt.getTable("StreamDeck");
        configuration.buttonConfigurations.forEach((button, selected) -> {
            var table = deckTable.getSubTable("Button/" + button.index);
            table.getStringTopic("Key").publish().set("/Dashboard/" + button.key);
            table.getStringTopic("Icon").publish().set(button.icon);
            table.getStringTopic("Label").publish().set(button.label);

            var dashboardBoolean = new LoggedNetworkButton(button.key, false);
            buttons.put(
                    button,
                    new Button(
                            dashboardBoolean,
                            selected.orElse(dashboardBoolean::get),
                            table.getBooleanTopic("Selected").publish()));
        });

        deckTable.getIntegerTopic("LastModified").publish().set(Logger.getTimestamp());

        return this;
    }

    public Trigger button(StreamDeckButton button) {
        if (!buttons.containsKey(button)) {
            if (!alerts.containsKey(button)) {
                var alert = new Alert("Stream Deck button trigger added for invalid button ", AlertType.kWarning);
                alerts.put(button, alert);
                alert.set(true);
            }
            return new Trigger(() -> false);
        }

        return new Trigger(buttons.get(button).pressed::get);
    }

    public ButtonGroup buttonGroup() {
        return new ButtonGroup();
    }

    public static enum StreamDeckButton {
        L4_CORAL(0, 0, "l4CoralScoreButton", "Circle", "L4"),
        L3_CORAL(1, 0, "l3CoralScoreButton", "Circle", "L3"),
        L2_CORAL(2, 0, "l2CoralScoreButton", "Circle", "L2"),
        L1_CORAL(2, 1, "l1CoralScoreButton", "Circle", "L1"),
        L3_ALGAE(0, 1, "l3AlgaeButton", "Circle", "L3 Algae"),
        L2_ALGAE(1, 1, "l2AlgaeButton", "Circle", "L2 Algae"),
        CORAL_INTAKE(2, 2, "coralIntakeButton", "Circle", "Coral In"),
        ALGAE_INTAKE(2, 3, "algaeIntakeButton", "Circle", "Algae In"),
        FEED(0, 2, "feedButton", "Circle", "Feed"),
        STOW(1, 2, "stowButton", "Circle", "Stow"),
        PROCESSOR(1, 3, "processorButton", "Circle", "Processor"),
        BARGE(0, 3, "bargeButton", "Circle", "Barge"),
        DEPLOY(0, 4, "deployClimbButton", "Circle", "Deploy"),
        RETRACT(1, 4, "retractClimbButton", "Circle", "Retract"),
        ZERO(2, 4, "zeroClimbButton", "Circle", "Zero");

        private final int index;
        private final String key;
        private final String icon;
        private final String label;

        private StreamDeckButton(int row, int col, String key, String icon, String label) {
            index = row * 5 + col % 5;
            this.key = key;
            this.icon = icon;
            this.label = label;
        }
    }

    public class ButtonConfiguration {
        private final Map<StreamDeckButton, Optional<BooleanSupplier>> buttonConfigurations = new HashMap<>();

        private ButtonConfiguration() {}

        public ButtonConfiguration addDefault(StreamDeckButton button) {
            buttonConfigurations.put(button, Optional.empty());
            return this;
        }

        public ButtonConfiguration add(StreamDeckButton button, BooleanSupplier selected) {
            buttonConfigurations.put(button, Optional.of(selected));
            return this;
        }
    }

    public class ButtonGroup {
        private final Map<StreamDeckButton, Trigger> triggers = new HashMap<>();
        private Optional<StreamDeckButton> selected = Optional.empty();

        private ButtonGroup() {}

        public ButtonGroup option(StreamDeckButton button) {
            initButton(button);
            return this;
        }

        public ButtonGroup option(StreamDeckButton button, Consumer<Trigger> trigger) {
            var buttonTrigger = initButton(button);
            trigger.accept(buttonTrigger);
            return this;
        }

        public ButtonGroup select(StreamDeckButton button) {
            selected = Optional.of(button);
            return this;
        }

        public ButtonGroup clear() {
            selected = Optional.empty();
            return this;
        }

        public Optional<StreamDeckButton> getSelected() {
            return selected;
        }

        public boolean isSelected(StreamDeckButton button) {
            return selected.isPresent() && selected.get() == button;
        }

        public Trigger trigger(StreamDeckButton button) {
            if (triggers.containsKey(button)) {
                return triggers.get(button);
            }

            return initButton(button);
        }

        private Trigger initButton(StreamDeckButton button) {
            button(button)
                    .onTrue(Commands.runOnce(() -> selected = Optional.of(button))
                            .ignoringDisable(true)
                            .withName("UpdateStreamDeckButtonGroupActiveIndex"));

            var trigger = new Trigger(() -> isSelected(button));
            triggers.put(button, trigger);
            return trigger;
        }
    }
}
