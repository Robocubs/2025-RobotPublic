package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
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
        this.goal = goal;
        if (goal == Goal.HOLD) {
            if (holdPosition.isEmpty()) {
                holdPosition = Optional.of(inputs.position);
            }

            io.setPosition(holdPosition.get());
            return;
        }

        holdPosition = Optional.empty();

        if (goal == Goal.STOP) {
            io.stop();
            return;
        }

        io.setPosition(goal.position);
    }

    public void stop() {
        setGoal(Goal.STOP);
    }

    @AutoLogOutput
    public boolean atGoal() {
        return atGoal(this.goal);
    }

    @AutoLogOutput
    public boolean atGoal(Goal goal) {
        return MathUtil.isNear(this.inputs.position, goal.position, positionTolerance);
    }
}
