package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public final class AutoScoreQuick {
    private static final double reefFaceLength = FieldConstants.reefFaceLength.in(Meters);

    public static final LoggedTunableMeasure<DistanceUnit, Distance> maxDistanceReefLineup =
            new LoggedTunableMeasure<>("AutoScore/MaxDistanceReefLineup", Meters.of(1.0));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> releaseTolerance =
            new LoggedTunableMeasure<>("AutoScore/ReleaseTolerance", Inches.of(1.1));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> slowModeDistance =
            new LoggedTunableMeasure<>("AutoScore/SlowModeDistance", Meters.of(0.15));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> minArticulationDistance =
            new LoggedTunableMeasure<>("AutoScore/ArticulationDistance", Meters.of(1.5));

    public static Command autoScoreQuick(
            Drive drive, Superstructure superstructure, RobotState robotState, Supplier<Optional<Pose2d>> goalPose) {
        var scheduler = new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        return deadline(autoScoreQuick(drive, scheduler, robotState, goalPose), scheduler.cmd());
    }

    public static Command autoScoreQuick(
            Drive drive, SubsystemScheduler<Superstructure> superstructure, RobotState robotState, Pose2d goalPose) {
        var optionalPose = Optional.of(goalPose);
        return autoScoreQuick(drive, superstructure, robotState, () -> optionalPose);
    }

    public static Command autoScoreQuick(
            Drive drive,
            SubsystemScheduler<Superstructure> superstructure,
            RobotState robotState,
            Supplier<Optional<Pose2d>> goalPose) {
        var autoScoreState = new AutoScoreQuickState();
        var updateState = runOnce(() -> {
            var scorePose = goalPose.get();
            if (scorePose.isEmpty()) {
                autoScoreState.valid = false;
                return;
            }

            autoScoreState.valid = true;
            autoScoreState.goalPose = scorePose.get();

            switch (robotState.getCoralSelection()) {
                case L4_CORAL:
                    autoScoreState.alignState = SuperstructureState.L4_CORAL;
                    autoScoreState.scoreState = SuperstructureState.L4_CORAL_SCORE;
                    break;
                case L3_CORAL:
                    autoScoreState.alignState = SuperstructureState.L3_CORAL;
                    autoScoreState.scoreState = SuperstructureState.L3_CORAL_SCORE;
                    break;
                case L2_CORAL:
                    autoScoreState.alignState = SuperstructureState.L2_CORAL;
                    autoScoreState.scoreState = SuperstructureState.L2_CORAL_SCORE;
                    break;
                case L1_CORAL:
                    autoScoreState.alignState = SuperstructureState.L1_CORAL;
                    autoScoreState.scoreState = SuperstructureState.L1_CORAL_SCORE;
                    break;
            }
        });

        var driveToGoalPose =
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoScoreState.goalPose), false, false);
        var driveToGoalPoseSlow = drive.toPose(() -> autoScoreState.goalPose, false, true);
        var stowSuperstructure =
                superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.STOW)));
        var feedSuperstructure =
                superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.FEED)));
        var score = superstructure.schedule(
                s -> s.defer(() -> s.score(autoScoreState.alignState, autoScoreState.scoreState, () -> robotState
                        .getDistanceTo(autoScoreState.goalPose)
                        .lt(releaseTolerance.get()))));

        return sequence(
                updateState,
                parallel(
                                sequence(
                                        driveToGoalPose.until(() -> robotState
                                                .getDistanceTo(autoScoreState.goalPose)
                                                .lt(slowModeDistance.get())),
                                        driveToGoalPoseSlow),
                                sequence(
                                        feedSuperstructure
                                                .until(() -> robotState.hasCoralLoaded())
                                                .unless(() -> robotState.hasCoralLoaded()),
                                        stowSuperstructure
                                                .until(() -> robotState
                                                        .getDistanceTo(autoScoreState.goalPose)
                                                        .lt(slowModeDistance.get()))
                                                .unless(() -> robotState
                                                        .getDistanceTo(autoScoreState.goalPose)
                                                        .lt(slowModeDistance.get())),
                                        score))
                        .unless(() -> !autoScoreState.valid));
    }

    private static Pose2d getTargetPose(Pose2d robot, Pose2d goal) {
        var offset = robot.relativeTo(goal);
        var yDistance = Math.abs(offset.getY());
        var xDistance = Math.abs(offset.getX());
        var shiftXT = MathUtil.clamp(
                (yDistance / (reefFaceLength * 2)) + ((xDistance - 0.3) / (reefFaceLength * 3)), 0.0, 1.0);
        var shiftYT = MathUtil.clamp(offset.getX() / reefFaceLength, 0.0, 1.0);
        return goal.transformBy(new Transform2d(
                -shiftXT * maxDistanceReefLineup.get().in(Meters),
                Math.copySign(shiftYT * maxDistanceReefLineup.get().in(Meters) * 0.8, offset.getY()),
                Rotation2d.kZero));
    }

    private static class AutoScoreQuickState {
        private boolean valid = false;
        private SuperstructureState alignState = SuperstructureState.L4_CORAL;
        private SuperstructureState scoreState = SuperstructureState.L4_CORAL_SCORE;
        private Pose2d goalPose = Pose2d.kZero;
    }
}
