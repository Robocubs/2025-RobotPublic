package frc.robot.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.wpilibj2.command.Commands.print;

public class Superstructure extends SubsystemBase {
    private final Elevator elevator;
    private final Arm arm;
    private final RobotState robotState;

    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public enum Goal {
        STOP,
        HOLD,
        STOW,
        RETRACT,
        L1_STOW_LONG,
        L1_STOW_WIDE,
        L1_SCORE_LONG,
        L1_SCORE_WIDE,
        L2_STOW,
        L3_STOW,
        L4_STOW,
        L2_INTAKE,
        L3_INTAKE,
        L4_SCORE,
        BARGE_STOW,
        BARGE_SCORE,
        ALGAE_INTAKE,
        CORAL_INTAKE_FLOOR,
        CORAL_INTAKE_FUNNEL,
        PROCESSOR
    }

    public Superstructure(ElevatorIO elevatorIO, ArmIO armIO, RobotState robotState) {
        elevator = new Elevator(elevatorIO);
        arm = new Arm(armIO);
        this.robotState = robotState;
    }

    @Override
    public void periodic() {
        elevator.periodic();
        arm.periodic();

        // TODO: Log mechanism 2d for the superstructure
    }

    @AutoLogOutput
    public boolean atGoal() {
        // TODO: Return whether the elevator and arm are both at the goal
        return false;
    }

    public void setGoal(Goal goal) {
        // TODO: Set the goal for the elevator and arm
    }

    public Command moveToGoal(Supplier<Goal> goal) {
        /*
         * TODO: Implement the following logic:
         * 1. If the superstructure is not at the goal, but the arm is extended, retract the arm while holding elevator position
         * 2. Move the elevator to the goal position
         * 3. Move to the final goal
         *
         * Examples:
         * - Moving from L2_INTAKE to L3_INTAKE
         *   1. Set elevator to hold position and arm to STOW
         *   2. Set elevator to L3
         *   3. Set arm to REEF_INTAKE
         * - Moving from STOW to FLOOR_INTAKE
         *   1. Set elevator to hold position and arm to STOW
         *   2. Set elevator to L3
         *   3. Set arm to REEF_INTAKE
         */
        return print("Moving superstructure to " + goal.get()).withName("SuperstructureSetGoal");
    }

    public Command hold() {
        // TODO: Hold the current position of the elevator and arm
        // Set the goal to hold
        return print("Holding superstructure").withName("SuperstructureHold");
    }

    public Command stop() {
        // TODO: Stop elevator and arm
        // Set the goal to stop
        return print("Stopping superstructure").withName("SuperstructureStop");
    }
}
