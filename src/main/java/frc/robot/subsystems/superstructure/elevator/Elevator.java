package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableMeasure;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class Elevator {
    private final LoggedTunableMeasure<VoltageUnit, Voltage> zeroRoutineVolts =
            new LoggedTunableMeasure<>("Elevator/ZeroRoutineVolts", Volts.of(-0.5));
    private final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> zeroVelocityTolerance =
            new LoggedTunableMeasure<>("Elevator/ZeroVelocityTolerance", MetersPerSecond.of(0.1));
    private final LoggedTunableMeasure<DistanceUnit, Distance> zeroPosition =
            new LoggedTunableMeasure<>("Elevator/ZeroPosition", Meters.of(-0.018));

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Optional<Distance> holdPosition = Optional.empty();

    @AutoLogOutput
    private Distance targetHeight = Meters.zero();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Distance getHeight() {
        return inputs.masterPosition;
    }

    public LinearVelocity getVelocity() {
        return inputs.masterVelocity;
    }

    public boolean isNear(Distance height) {
        return this.inputs.masterPosition.isNear(height, positionTolerance);
    }

    @AutoLogOutput
    public boolean atTarget() {
        return isNear(targetHeight);
    }

    public void setHeight(Distance height) {
        holdPosition = Optional.empty();
        targetHeight = height;
        io.setPosition(height);
    }

    public void setHeight(Distance height, Force feedforward) {
        holdPosition = Optional.empty();
        targetHeight = height;
        if (height.isEquivalent(Meters.zero())) {
            io.stop();
        } else {
            io.setPosition(height, feedforward);
        }
    }

    public void setVelocity(LinearVelocity velocity, Force feedforward) {
        holdPosition = Optional.empty();
        targetHeight = inputs.masterPosition.plus(velocity.times(Constants.mainLoopPeriod));
        io.setVelocity(velocity, feedforward);
    }

    public void stop() {
        holdPosition = Optional.empty();
        io.stop();
    }

    public void hold() {
        if (holdPosition.isEmpty()) {
            holdPosition = Optional.of(inputs.masterPosition);
        }

        targetHeight = holdPosition.get();
        io.setPosition(holdPosition.get());
    }

    public void runCharacterization(Current current) {
        holdPosition = Optional.empty();
        io.setTorqueCurrent(current);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        io.setNeutralMode(neutralMode);
    }

    public Command setZeroPosition() {
        return setZeroPosition(Meters.zero());
    }

    public Command setZeroPosition(Distance height) {
        return Commands.run(() -> {
                    io.stop();
                    io.zeroPosition(height);
                })
                .until(() -> isNear(height));
    }

    public Command zeroRoutine() {
        var debouncer = new Debouncer(0.5, DebounceType.kRising);
        return Commands.runOnce(() -> debouncer.calculate(false))
                .andThen(Commands.run(() -> io.setVoltage(zeroRoutineVolts.get())))
                .until(() -> debouncer.calculate(
                        inputs.masterVelocity.isNear(MetersPerSecond.zero(), zeroVelocityTolerance.get())))
                .andThen(setZeroPosition(zeroPosition.get()));
    }
}
