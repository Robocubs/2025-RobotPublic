package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class Elevator {
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
        return Commands.run(() -> {
                    io.stop();
                    io.zeroPosition();
                })
                .until(() -> isNear(Meters.zero()));
    }

    public Command zeroRoutine() {
        var voltage = Volts.of(-1.0);
        var maxCurrent = Amps.of(-60);
        return Commands.run(() -> io.setVoltage(voltage))
                .until(() -> inputs.masterTorqueCurrent.lt(maxCurrent))
                .andThen(setZeroPosition());
    }
}
