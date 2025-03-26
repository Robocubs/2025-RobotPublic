package frc.robot.subsystems.superstructure.controllers;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

/**
 * A simple controller that moves the superstructure to the desired goal.
 * It will retract the arm, move the elevator, then move the arm to the desired angle.
 */
public class SimpleController implements SuperstructureController {
    private final Superstructure superstructure;

    private final Map<SuperstructureState, SuperstructureState> scoringStateMoves = Map.ofEntries(
            Map.entry(SuperstructureState.L1_CORAL_SCORE, SuperstructureState.L1_CORAL),
            Map.entry(SuperstructureState.L2_CORAL_SCORE, SuperstructureState.L2_CORAL),
            Map.entry(SuperstructureState.L3_CORAL_SCORE, SuperstructureState.L3_CORAL),
            Map.entry(SuperstructureState.L4_CORAL_SCORE, SuperstructureState.L4_CORAL),
            Map.entry(SuperstructureState.BARGE_SCORE, SuperstructureState.BARGE),
            Map.entry(SuperstructureState.PROCESSOR_SCORE, SuperstructureState.PROCESSOR));

    private final Map<SuperstructureState, SuperstructureState> retractingStateMoves = Map.ofEntries(
            Map.entry(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_RETRACTED),
            Map.entry(SuperstructureState.L4_CORAL_SCORE, SuperstructureState.L4_CORAL_RETRACTED),
            Map.entry(SuperstructureState.BARGE, SuperstructureState.BARGE_RETRACTED),
            Map.entry(SuperstructureState.BARGE_SCORE, SuperstructureState.BARGE_RETRACTED),
            Map.entry(SuperstructureState.CORAL_INTAKE_1, SuperstructureState.CORAL_INTAKE_1_RETRACTED),
            Map.entry(SuperstructureState.CORAL_INTAKE_2, SuperstructureState.CORAL_INTAKE_1_RETRACTED),
            Map.entry(SuperstructureState.L2_ALGAE, SuperstructureState.L2_ALGAE_RETRACTED),
            Map.entry(SuperstructureState.L3_ALGAE, SuperstructureState.L3_ALGAE_RETRACTED));

    public SimpleController(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public Command getCommand(SuperstructureState start, SuperstructureState goal) {
        var retractFromPreviousState =
                runState(() -> retractingStateMoves.get(start)).onlyIf(() -> retractingStateMoves.containsKey(start));
        var moveToRetractedState =
                runState(() -> retractingStateMoves.get(goal)).onlyIf(() -> retractingStateMoves.containsKey(goal));
        var moveToScoringState =
                runState(() -> scoringStateMoves.get(goal)).onlyIf(() -> scoringStateMoves.containsKey(goal));

        return Commands.sequence(retractFromPreviousState, moveToRetractedState, moveToScoringState)
                .onlyIf(() -> !canImmediatelyRunState(start, goal))
                .andThen(runState(() -> goal));
    }

    private boolean canImmediatelyRunState(SuperstructureState start, SuperstructureState goal) {
        var currentState = start;
        if (goal == start) {
            return true;
        }

        if (currentState == SuperstructureState.CORAL_INTAKE_1
                        && superstructure.isNear(SuperstructureState.CORAL_INTAKE_2)
                || currentState == SuperstructureState.CORAL_INTAKE_2
                        && superstructure.isNear(SuperstructureState.CORAL_INTAKE_1)) {
            return true;
        }

        return false;
    }

    private Command runState(Supplier<SuperstructureState> state) {
        return superstructure
                .run(() -> superstructure.runStatePeriodic(state.get()))
                .until(() -> superstructure.atStatePose());
    }
}
