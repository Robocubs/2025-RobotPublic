package frc.robot.subsystems.superstructure.controllers;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

public class GraphController implements SuperstructureController {
    private final Superstructure superstructure;
    private final Graph<SuperstructureState, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    public GraphController(Superstructure superstructure) {
        this.superstructure = superstructure;

        for (var goal : SuperstructureState.values()) {
            graph.addVertex(goal);
        }

        var specialStates = Set.of(SuperstructureState.START, SuperstructureState.STOP, SuperstructureState.HOLD);

        var freeStates = Set.of(
                SuperstructureState.STOW,
                SuperstructureState.FEED_RETRACTED,
                SuperstructureState.L1_CORAL_WIDE,
                SuperstructureState.L1_CORAL,
                SuperstructureState.L2_CORAL,
                SuperstructureState.L3_CORAL,
                SuperstructureState.L4_CORAL_RETRACTED,
                SuperstructureState.BARGE_RETRACTED,
                SuperstructureState.L2_ALGAE_RETRACTED,
                SuperstructureState.L3_ALGAE_RETRACTED,
                SuperstructureState.ALGAE_INTAKE,
                SuperstructureState.PROCESSOR,
                SuperstructureState.CORAL_INTAKE_1_RETRACTED);

        // Special states to free states
        for (var state : specialStates) {
            graph.addEdge(state, SuperstructureState.RETRACT_ARM);
        }

        for (var state : freeStates) {
            graph.addEdge(SuperstructureState.RETRACT_ARM, state);
        }

        // Free states to other free states
        for (var from : freeStates) {
            for (var to : freeStates) {
                if (from == to) {
                    continue;
                }

                graph.addEdge(from, to, getEdgeCommand(from, to));
            }
        }

        // To/from retracting states
        var retractingStateMoves = Map.ofEntries(
                Map.entry(SuperstructureState.FEED, SuperstructureState.FEED_RETRACTED),
                Map.entry(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_RETRACTED),
                Map.entry(SuperstructureState.L4_CORAL_SCORE, SuperstructureState.L4_CORAL_RETRACTED),
                Map.entry(SuperstructureState.BARGE, SuperstructureState.BARGE_RETRACTED),
                Map.entry(SuperstructureState.CORAL_INTAKE_1, SuperstructureState.CORAL_INTAKE_1_RETRACTED),
                Map.entry(SuperstructureState.L2_ALGAE, SuperstructureState.L2_ALGAE_RETRACTED),
                Map.entry(SuperstructureState.L3_ALGAE, SuperstructureState.L3_ALGAE_RETRACTED));

        for (var entry : retractingStateMoves.entrySet()) {
            graph.addEdge(entry.getKey(), entry.getValue());
            graph.addEdge(entry.getValue(), entry.getKey());
        }

        var scoringStates = Map.ofEntries(
                Map.entry(SuperstructureState.L1_CORAL_WIDE_SCORE, SuperstructureState.L1_CORAL_WIDE),
                Map.entry(SuperstructureState.L1_CORAL_SCORE, SuperstructureState.L1_CORAL),
                Map.entry(SuperstructureState.L2_CORAL_SCORE, SuperstructureState.L2_CORAL),
                Map.entry(SuperstructureState.L3_CORAL_SCORE, SuperstructureState.L3_CORAL),
                Map.entry(SuperstructureState.L4_CORAL_SCORE, SuperstructureState.L4_CORAL),
                Map.entry(SuperstructureState.BARGE_SCORE, SuperstructureState.BARGE),
                Map.entry(SuperstructureState.PROCESSOR_SCORE, SuperstructureState.PROCESSOR));

        // To/from scoring states
        for (var entry : scoringStates.entrySet()) {
            graph.addEdge(entry.getKey(), entry.getValue());
            graph.addEdge(entry.getValue(), entry.getKey());
        }

        // Floor states
        graph.addEdge(SuperstructureState.CORAL_INTAKE_1_RETRACTED, SuperstructureState.CORAL_INTAKE_1);
        graph.addEdge(SuperstructureState.CORAL_INTAKE_1, SuperstructureState.CORAL_INTAKE_1_RETRACTED);
        graph.addEdge(SuperstructureState.CORAL_INTAKE_1, SuperstructureState.CORAL_INTAKE_2);
        graph.addEdge(SuperstructureState.CORAL_INTAKE_2, SuperstructureState.CORAL_INTAKE_1);
    }

    @Override
    public Command getCommand(SuperstructureState start, SuperstructureState goal) {
        return bfs(start, goal).orElse(Commands.print("No path found between " + start + " and " + goal));
    }

    private Optional<Command> bfs(SuperstructureState start, SuperstructureState goal) {
        // Map to track the parent of each visited node
        Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
        Queue<SuperstructureState> queue = new LinkedList<>();
        queue.add(start);
        parents.put(start, null); // Mark the start node as visited with no parent

        // Perform BFS
        while (!queue.isEmpty()) {
            var current = queue.poll();
            // Check if we've reached the goal
            if (current == goal) {
                break;
            }
            // Process valid neighbors
            for (var edge : graph.outgoingEdgesOf(current).stream()
                    .filter(edge -> isEdgeAllowed(edge, goal))
                    .toList()) {
                var neighbor = graph.getEdgeTarget(edge);
                // Only process unvisited neighbors
                if (!parents.containsKey(neighbor)) {
                    parents.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        // Reconstruct the path to the goal if found
        if (!parents.containsKey(goal)) {
            return Optional.empty(); // Goal not reachable
        }

        // Trace back the path from goal to start
        var command = runState(goal);
        var nextState = goal;
        while (!nextState.equals(start)) {
            var parent = parents.get(nextState);
            if (parent == null) {
                return Optional.empty(); // Should never be null
            }

            command = runState(parent).andThen(command);
            nextState = parent;
        }
        return Optional.of(command);
    }

    private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
        return true;
    }

    private EdgeCommand getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        return EdgeCommand.builder().command(runState(to)).build();
    }

    private Command runState(SuperstructureState state) {
        return superstructure
                .run(() -> superstructure.runStatePeriodic(state))
                .until(() -> superstructure.isNear(state));
    }

    @AllArgsConstructor
    @Builder(toBuilder = true)
    @Getter
    public static class EdgeCommand extends DefaultEdge {
        public EdgeCommand() {
            this.command = null;
        }

        private final Command command;
    }
}
