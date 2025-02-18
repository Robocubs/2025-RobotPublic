package frc.robot.subsystems.superstructure.rollers;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.booleans.LatchedBoolean;
import frc.robot.util.booleans.ThresholdLatchedBoolean;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;

public class Rollers {
    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private final ThresholdLatchedBoolean longCoralDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            coralDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final ThresholdLatchedBoolean algaeDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            algaeDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final LatchedBoolean wideCoralDetected = new LatchedBoolean(true);

    private Optional<Angle> autoFeedCoralCoralPosition = Optional.empty();
    private Optional<Angle> autoFeedCoralHybridPosition = Optional.empty();
    private Optional<Angle> autoIntakeCoralCoralPosition = Optional.empty();
    private Optional<Angle> autoIntakeCoralHybridPosition = Optional.empty();
    private Optional<Angle> autoIntakeAlgaeHybridPosition = Optional.empty();

    private @AutoLogOutput State state = State.STOPPED;

    public static enum State {
        STOPPED,
        AUTO_FEED_CORAL,
        AUTO_INTAKE_CORAL,
        AUTO_INTAKE_ALGAE,
        CORAL_FORWARD,
        CORAL_REVERSE,
        ALGAE_FORWARD,
        HOLD
    }

    public Rollers(RollersIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);

        longCoralDetected.update(inputs.coralDetectorDistance.in(Meters));
        algaeDetected.update(inputs.algaeDetectorDistance.in(Meters));

        if (state == State.AUTO_INTAKE_CORAL) {
            wideCoralDetected.update(inputs.coralDetectorDistance.lt(coralDetectionDistance));
        } else if (state == State.CORAL_FORWARD || state == State.AUTO_INTAKE_ALGAE) {
            wideCoralDetected.resetLatch();
        }
    }

    public void runState(State state) {
        if (wideCoralDetected.get()) {
            if (!autoIntakeCoralCoralPosition.isPresent() || !autoIntakeCoralHybridPosition.isPresent()) {
                autoIntakeCoralCoralPosition = Optional.of(inputs.coralPosition.plus(coralIntakeCoralRollerPosition));
                autoIntakeCoralHybridPosition =
                        Optional.of(inputs.hybridPosition.plus(coralIntakeHybridRollerPosition));
            }
        } else {
            autoIntakeCoralCoralPosition = Optional.empty();
            autoIntakeCoralHybridPosition = Optional.empty();
        }

        if (longCoralDetected.get()) {
            if (!autoFeedCoralCoralPosition.isPresent() || autoIntakeCoralHybridPosition.isPresent()) {
                autoFeedCoralCoralPosition = Optional.of(inputs.coralPosition.minus(coralFeedCoralRollerPosition));
                autoFeedCoralHybridPosition = Optional.of(inputs.hybridPosition.minus(coralFeedHybridRollerPosition));
            }
        } else {
            autoFeedCoralCoralPosition = Optional.empty();
            autoFeedCoralHybridPosition = Optional.empty();
        }

        if (algaeDetected.get()) {
            if (!autoIntakeAlgaeHybridPosition.isPresent()) {
                autoIntakeAlgaeHybridPosition =
                        Optional.of(inputs.hybridPosition.plus(algaeIntakeHybridRollerPosition));
            }
        } else {
            autoIntakeAlgaeHybridPosition = Optional.empty();
        }

        switch (state) {
            case AUTO_FEED_CORAL:
                if (autoFeedCoralCoralPosition.isPresent() && autoFeedCoralHybridPosition.isPresent()) {
                    io.setCoralPosition(autoFeedCoralCoralPosition.get());
                    io.setHybridPosition(autoFeedCoralHybridPosition.get());
                } else {
                    io.setCoralVelocity(coralFeedCoralRollerVelocity);
                    io.setHybridVelocity(coralFeedHybridRollerVelocity);
                }
                break;
            case AUTO_INTAKE_CORAL:
                if (autoIntakeCoralCoralPosition.isPresent() && autoIntakeCoralHybridPosition.isPresent()) {
                    io.setCoralPosition(autoIntakeCoralCoralPosition.get());
                    io.setHybridPosition(autoIntakeCoralHybridPosition.get());
                } else {
                    io.setCoralVelocity(coralIntakeCoralRollerVelocity);
                    io.setHybridVelocity(coralIntakeHybridRollerVelocity);
                }
                break;
            case AUTO_INTAKE_ALGAE:
                if (autoIntakeAlgaeHybridPosition.isPresent() && autoIntakeCoralHybridPosition.isPresent()) {
                    io.stopCoral();
                    io.setHybridPosition(autoIntakeAlgaeHybridPosition.get());
                } else {
                    io.setCoralVelocity(algaeIntakeCoralRollerVelocity);
                    io.setHybridVelocity(algaeIntakeHybridRollerVelocity);
                }
                break;
            case CORAL_FORWARD:
                io.setCoralVelocity(algaeIntakeCoralRollerVelocity);
                break;
            case CORAL_REVERSE:
                io.setCoralVelocity(algaeIntakeCoralRollerVelocity);
                break;
            case ALGAE_FORWARD:
                io.setCoralVelocity(algaeIntakeCoralRollerVelocity);
                break;
            case HOLD:
                if (autoFeedCoralCoralPosition.isPresent()) {
                    io.setCoralPosition(autoFeedCoralCoralPosition.get());
                } else if (autoIntakeCoralCoralPosition.isPresent()) {
                    io.setCoralPosition(autoIntakeCoralCoralPosition.get());
                } else if (autoIntakeAlgaeHybridPosition.isPresent()) {
                    io.setCoralPosition(autoIntakeAlgaeHybridPosition.get());
                } else {
                    io.stopCoral();
                }
                break;
            case STOPPED:
                io.stopCoral();
                break;
            default:
                io.stopCoral();
                break;
        }
    }

    @AutoLogOutput
    public boolean longCoralDetected() {
        return longCoralDetected.get();
    }

    @AutoLogOutput
    public boolean wideCoralDetected() {
        return wideCoralDetected.get();
    }

    @AutoLogOutput
    public boolean algaeDetected() {
        return algaeDetected.get();
    }

    public void stop() {
        io.stopCoral();
        autoFeedCoralCoralPosition = Optional.empty();
        autoIntakeCoralCoralPosition = Optional.empty();
        autoIntakeAlgaeHybridPosition = Optional.empty();
        state = State.STOPPED;
    }
}
