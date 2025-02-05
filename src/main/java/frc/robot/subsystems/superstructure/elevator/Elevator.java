package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
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

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Distance getHeight() {
        return inputs.position;
    }

    public boolean isNear(Distance height) {
        return this.inputs.position.isNear(height, positionTolerance);
    }

    public void setHeight(Distance height) {
        holdPosition = Optional.empty();
        targetHeight = height;
        io.setPosition(height);
    }

    public void setHeight(Distance height, Force feedforward) {
        holdPosition = Optional.empty();
        targetHeight = height;
        io.setPosition(height, feedforward);
    }

    public void setVelocity(LinearVelocity velocity, Force feedforward) {
        holdPosition = Optional.empty();
        targetHeight = inputs.position.plus(velocity.times(Constants.mainLoopPeriod));
        io.setVelocity(velocity, feedforward);
    }

    public void stop() {
        holdPosition = Optional.empty();
        io.stop();
    }

    public void hold() {
        if (holdPosition.isEmpty()) {
            holdPosition = Optional.of(inputs.position);
        }

        targetHeight = holdPosition.get();
        io.setPosition(holdPosition.get());
    }
}
