package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Optional<Double> holdPosition = Optional.empty();

    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public enum Goal {
        STOP(),
        HOLD(),
        STOW(0.0),
        L1_LONG(0.5),
        L1_WIDE(0.2),
        L2_CORAL(1.0),
        L2_ALGAE(1.0),
        L3_CORAL(2.0),
        L3_ALGAE(2.0),
        L4(3.5),
        BARGE(4.5),
        ALGAE_INTAKE(0.1),
        CORAL_INTAKE_FLOOR(0.3),
        CORAL_INTAKE_FUNNEL(0.0);

        private final double position;

        private Goal() {
            this.position = -1.0;
        }

        private Goal(double position) {
            this.position = position;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setGoal(Goal goal) {
        /*
         * TODO: Control the elevator to move to the specified goal
         * 1. Set the goal to the specified goal
         * 2. Set the position on the IO
         *
         * If the goal is HOLD, set the position to the current measured value.
         * Make sure that this position does not change if the goal is already HOLD.
         */
    }

    public void stop() {
        // TODO: Stop the elevator from moving
        // Set the goal to STOP
    }

    @AutoLogOutput
    public boolean atGoal() {
        // TODO: Return whether the elevator is at the goal within the tolerance set in ElevatorConstants
        return false;
    }

    @AutoLogOutput
    public boolean atGoal(Goal goal) {
        // TODO: Return whether the elevator is at the supplied goal within the tolerance set in ElevatorConstants
        return false;
    }
}
