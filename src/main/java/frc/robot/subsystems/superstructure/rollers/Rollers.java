package frc.robot.subsystems.superstructure.rollers;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.booleans.LatchedBoolean;
import frc.robot.util.booleans.ThresholdLatchedBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;

public class Rollers {
    private static final LoggedTunableNumber coralFeedDistance =
            new LoggedTunableNumber("Rollers/CoralFeedDistance", RollersConstants.coralFeedDistance.in(Meters));
    private static final LoggedTunableNumber coralIntakeDistance =
            new LoggedTunableNumber("Rollers/CoralIntakeDistance", RollersConstants.coralIntakeDistance.in(Meters));
    private static final LoggedTunableNumber algaeIntakeDistance =
            new LoggedTunableNumber("Rollers/AlgaeIntakeDistance", RollersConstants.algaeIntakeDistance.in(Meters));

    private static final Angle positionTolerance = Radians.of(0.1);

    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private final ThresholdLatchedBoolean longCoralDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            coralDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final ThresholdLatchedBoolean algaeDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            algaeDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final LatchedBoolean wideCoralDetected = new LatchedBoolean(true);

    private Angle coralFeedCoralRollerPosition = Radians.of(coralFeedDistance.get() / coralRollerRadius.in(Meters));
    private Angle coralFeedHybridRollerPosition = Radians.of(coralFeedDistance.get() / hybridRollerRadius.in(Meters));
    private Angle coralIntakeCoralRollerPosition = Radians.of(coralIntakeDistance.get() / coralRollerRadius.in(Meters));
    private Angle coralIntakeHybridRollerPosition =
            Radians.of(coralIntakeDistance.get() / hybridRollerRadius.in(Meters));
    private Angle algaeIntakeHybridRollerPosition =
            Radians.of(algaeIntakeDistance.get() / hybridRollerRadius.in(Meters));

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
            wideCoralDetected.update(inputs.coralDetectorDistance.lt(coralDetectionDistance));
        }

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    coralFeedCoralRollerPosition = Radians.of(coralFeedDistance.get() / coralRollerRadius.in(Meters));
                    coralFeedHybridRollerPosition = Radians.of(coralFeedDistance.get() / hybridRollerRadius.in(Meters));
                    coralIntakeCoralRollerPosition =
                            Radians.of(coralIntakeDistance.get() / coralRollerRadius.in(Meters));
                    coralIntakeHybridRollerPosition =
                            Radians.of(coralIntakeDistance.get() / hybridRollerRadius.in(Meters));
                    algaeIntakeHybridRollerPosition =
                            Radians.of(algaeIntakeDistance.get() / hybridRollerRadius.in(Meters));
                },
                coralFeedDistance,
                coralIntakeDistance,
                algaeIntakeDistance);
    }

    public void runState(State state) {
        this.state = state;

        if (wideCoralDetected.get()) {
            if (autoIntakeCoralCoralPosition.isEmpty() || autoIntakeCoralHybridPosition.isEmpty()) {
                autoIntakeCoralCoralPosition = Optional.of(inputs.coralPosition.plus(coralIntakeCoralRollerPosition));
                autoIntakeCoralHybridPosition =
                        Optional.of(inputs.hybridPosition.plus(coralIntakeHybridRollerPosition));
            }
        } else {
            autoIntakeCoralCoralPosition = Optional.empty();
            autoIntakeCoralHybridPosition = Optional.empty();
        }

        if (longCoralDetected.get()) {
            if (autoFeedCoralCoralPosition.isEmpty() || autoFeedCoralHybridPosition.isEmpty()) {
                autoFeedCoralCoralPosition = Optional.of(inputs.coralPosition.plus(coralFeedCoralRollerPosition));
                autoFeedCoralHybridPosition = Optional.of(inputs.hybridPosition.plus(coralFeedHybridRollerPosition));
            }
        } else {
            autoFeedCoralCoralPosition = Optional.empty();
            autoFeedCoralHybridPosition = Optional.empty();
        }

        if (algaeDetected.get()) {
            if (autoIntakeAlgaeHybridPosition.isEmpty()) {
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
                io.setCoralVelocity(coralForwardCoralRollerVelocity);
                io.setHybridVelocity(coralForwardHybridRollerVelocity);
                break;
            case CORAL_REVERSE:
                io.setCoralVelocity(coralReverseCoralRollerVelocity);
                io.setHybridVelocity(coralReverseHybridRollerVelocity);
                break;
            case ALGAE_FORWARD:
                io.setCoralVelocity(algaeForwardCoralRollerVelocity);
                io.setHybridVelocity(algaeForwardHybridRollerVelocity);
                break;
            case HOLD:
                if (autoFeedCoralCoralPosition.isPresent() && autoFeedCoralHybridPosition.isPresent()) {
                    io.setCoralPosition(autoFeedCoralCoralPosition.get());
                    io.setHybridPosition(autoFeedCoralHybridPosition.get());
                } else if (autoIntakeCoralCoralPosition.isPresent() && autoIntakeCoralHybridPosition.isPresent()) {
                    io.setCoralPosition(autoIntakeCoralCoralPosition.get());
                    io.setHybridPosition(autoIntakeCoralHybridPosition.get());
                } else if (autoIntakeAlgaeHybridPosition.isPresent()) {
                    io.stopCoral();
                    io.setCoralPosition(autoIntakeAlgaeHybridPosition.get());
                } else {
                    io.stopCoral();
                    io.stopHybrid();
                }
                break;
            case STOPPED:
                io.stopCoral();
                io.stopHybrid();
                break;
            default:
                io.stopCoral();
                io.stopHybrid();
                break;
        }
    }

    @AutoLogOutput
    public boolean atAutoFeedPosition() {
        return autoFeedCoralCoralPosition.isPresent()
                && autoFeedCoralHybridPosition.isPresent()
                && inputs.coralPosition.isNear(autoFeedCoralCoralPosition.get(), positionTolerance)
                && inputs.hybridPosition.isNear(autoFeedCoralHybridPosition.get(), positionTolerance);
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

    public Command bumpFeedPosition(Distance distance) {
        return runOnce(() -> {
            if (autoFeedCoralCoralPosition.isPresent() && autoFeedCoralHybridPosition.isPresent()) {
                autoFeedCoralCoralPosition = Optional.of(autoFeedCoralCoralPosition
                        .get()
                        .plus(Radians.of(distance.in(Meters) / coralRollerRadius.in(Meters))));
                autoFeedCoralHybridPosition = Optional.of(autoFeedCoralHybridPosition
                        .get()
                        .plus(Radians.of(distance.in(Meters) / hybridRollerRadius.in(Meters))));
            }
        });
    }

    public void stop() {
        io.stopCoral();
        autoFeedCoralCoralPosition = Optional.empty();
        autoIntakeCoralCoralPosition = Optional.empty();
        autoIntakeAlgaeHybridPosition = Optional.empty();
        state = State.STOPPED;
    }
}
