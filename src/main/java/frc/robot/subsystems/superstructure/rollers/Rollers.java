package frc.robot.subsystems.superstructure.rollers;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.booleans.ThresholdLatchedBoolean;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableValue;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;

public class Rollers {
    private static final LoggedTunableMeasure<DistanceUnit, Distance> coralFeedDistance =
            new LoggedTunableMeasure<>("Rollers/CoralFeedDistance", Meters.of(0.05));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> coralIntakeDistance =
            new LoggedTunableMeasure<>("Rollers/CoralIntakeDistance", Meters.of(0.05));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> algaeIntakeDistance =
            new LoggedTunableMeasure<>("Rollers/AlgaeIntakeDistance", Meters.of(0.05));
    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> coralFeedVelocity =
            new LoggedTunableMeasure<>("Rollers/CoralFeedVelocity", maxAngularVelocity.times(0.3));
    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> coralFastFeedVelocity =
            new LoggedTunableMeasure<>("Rollers/CoralFastFeedVelocity", maxAngularVelocity.times(0.4));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> coralIntakeVelocity =
            new LoggedTunableMeasure<>("Rollers/CoralIntakeVelocity", maxAngularVelocity.times(0.5));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> coralForwardVelocity =
            new LoggedTunableMeasure<>("Rollers/CoralForwardVelocity", maxAngularVelocity.times(0.4));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> coralReverseVelocity =
            new LoggedTunableMeasure<>("Rollers/CoralReverseVelocity", maxAngularVelocity.times(-0.5));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> algaeIntakeVelocity =
            new LoggedTunableMeasure<>("Rollers/AlgaeIntakeVelocity", maxAngularVelocity.times(0.5));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> algaeForwardVelocity =
            new LoggedTunableMeasure<>("Rollers/AlgaeForwardVelocity", maxAngularVelocity.times(-0.5));

    private static final Angle positionTolerance = Radians.of(0.2);

    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private final ThresholdLatchedBoolean coralDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            coralDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final ThresholdLatchedBoolean algaeDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            algaeDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final ThresholdLatchedBoolean elevatorDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            elevatorDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final ThresholdLatchedBoolean funnelDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            funnelDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final Supplier<Distance> elevatorHeight;

    private Angle coralFeedRollerPosition =
            Radians.of(coralFeedDistance.get().in(Meters) / coralRollerRadius.in(Meters));
    private Angle coralIntakeRollerPosition =
            Radians.of(coralIntakeDistance.get().in(Meters) / coralRollerRadius.in(Meters));
    private Angle algaeIntakeRollerPosition =
            Radians.of(algaeIntakeDistance.get().in(Meters) / algaeRollerRadius.in(Meters));

    private Optional<Angle> autoRollerPosition = Optional.empty();

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

    public Rollers(RollersIO io, Supplier<Distance> elevatorHeight) {
        this.io = io;
        this.elevatorHeight = elevatorHeight;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);

        coralDetected.update(inputs.coralDetectorDistance.in(Meters));
        algaeDetected.update(inputs.algaeDetectorDistance.in(Meters));
        funnelDetected.update(inputs.funnelDetectorDistance.in(Meters));

        if (elevatorHeight.get().lt(elevatorMaxHeightForDetection)) {
            elevatorDetected.update(inputs.elevatorDetectorDistance.in(Meters));
        } else {
            elevatorDetected.update(Double.POSITIVE_INFINITY);
        }

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    coralFeedRollerPosition =
                            Radians.of(coralFeedDistance.get().in(Meters) / coralRollerRadius.in(Meters));
                    coralIntakeRollerPosition =
                            Radians.of(coralIntakeDistance.get().in(Meters) / coralRollerRadius.in(Meters));
                    algaeIntakeRollerPosition =
                            Radians.of(algaeIntakeDistance.get().in(Meters) / algaeRollerRadius.in(Meters));
                },
                coralFeedDistance,
                coralIntakeDistance,
                algaeIntakeDistance);
    }

    public void runState(State state) {
        this.state = state;

        if (coralDetected.get() && state != State.AUTO_INTAKE_ALGAE) {
            if (autoRollerPosition.isEmpty()) {
                autoRollerPosition = Optional.of(
                        state == State.AUTO_INTAKE_CORAL
                                ? inputs.position.plus(coralIntakeRollerPosition)
                                : inputs.position.plus(coralFeedRollerPosition));
            }
        } else if (algaeDetected.get() && state != State.AUTO_INTAKE_CORAL) {
            if (autoRollerPosition.isEmpty()) {
                autoRollerPosition = Optional.of(inputs.position.plus(algaeIntakeRollerPosition));
            }
        } else {
            autoRollerPosition = Optional.empty();
        }

        switch (state) {
            case AUTO_FEED_CORAL:
                if (autoRollerPosition.isPresent()) {
                    io.setPosition(autoRollerPosition.get());
                } else if (coralDetected.get()) {
                    io.setVelocity(coralFastFeedVelocity.get());
                } else {
                    io.setVelocity(coralFeedVelocity.get());
                }
                break;
            case AUTO_INTAKE_CORAL:
                if (autoRollerPosition.isPresent()) {
                    io.setPosition(autoRollerPosition.get());
                } else {
                    io.setVelocity(coralIntakeVelocity.get());
                }
                break;
            case AUTO_INTAKE_ALGAE:
                if (autoRollerPosition.isPresent()) {
                    io.setPosition(autoRollerPosition.get());
                } else {
                    io.setVelocity(algaeIntakeVelocity.get());
                }
                break;
            case CORAL_FORWARD:
                io.setVelocity(coralForwardVelocity.get());
                break;
            case CORAL_REVERSE:
                io.setVelocity(coralReverseVelocity.get());
                break;
            case ALGAE_FORWARD:
                io.setVelocity(algaeForwardVelocity.get());
                break;
            case HOLD:
                if (autoRollerPosition.isPresent()) {
                    io.setPosition(autoRollerPosition.get());
                } else {
                    io.stop();
                }
                break;
            case STOPPED:
                io.stop();
                break;
            default:
                io.stop();
                break;
        }
    }

    @AutoLogOutput
    public boolean atAutoFeedPosition() {
        return autoRollerPosition.isPresent() && inputs.position.isNear(autoRollerPosition.get(), positionTolerance);
    }

    @AutoLogOutput
    public boolean coralDetected() {
        return coralDetected.get() && inputs.coralSignalStrengthSignal > 2500;
    }

    @AutoLogOutput
    public boolean algaeDetected() {
        return algaeDetected.get() && inputs.algaeSignalStrengthSignal > 2500;
    }

    @AutoLogOutput
    public boolean elevatorDetected() {
        return elevatorDetected.get() && inputs.elevatorSignalStrength > 2500;
    }

    @AutoLogOutput
    public boolean funnelDetected() {
        return funnelDetected.get() && inputs.funnelSignalStrength > 2500;
    }

    public Command bumpFeedPosition(Distance distance) {
        return runOnce(() -> {
            if (autoRollerPosition.isPresent()) {
                autoRollerPosition = Optional.of(
                        autoRollerPosition.get().plus(Radians.of(distance.in(Meters) / coralRollerRadius.in(Meters))));
            }
        });
    }

    public void stop() {
        io.stop();
        autoRollerPosition = Optional.empty();
        state = State.STOPPED;
    }
}
