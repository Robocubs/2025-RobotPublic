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

public final class AutoScoreV2 {
    private static final double reefFaceLength = FieldConstants.reefFaceLength.in(Meters);

    public static final LoggedTunableMeasure<DistanceUnit, Distance> maxDistanceReefLineup =
            new LoggedTunableMeasure<>("AutoScoreV2/MaxDistanceReefLineup", Meters.of(1.0));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> alignDistance =
            new LoggedTunableMeasure<>("AutoScoreV2/AlignDistance", Meters.of(0.15));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> bounceDistance =
            new LoggedTunableMeasure<>("AutoScoreV2/BounceDistance", Meters.of(0.05));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> reverseDistance =
            new LoggedTunableMeasure<>("AutoScoreV2/ReverseDistance", Meters.of(0.5));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> releaseTolerance =
            new LoggedTunableMeasure<>("AutoScoreV2/ReleaseTolerance", Meters.of(0.05));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> minArticulationDistance =
            new LoggedTunableMeasure<>("AutoScoreV2/ArticulationDistance", Meters.of(1.5));

    public static Command autoScore(
            Drive drive, Superstructure superstructure, RobotState robotState, Supplier<Optional<Pose2d>> goalPose) {
        var scheduler = new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        return deadline(autoScore(drive, scheduler, robotState, goalPose), scheduler.cmd());
    }

    public static Command autoScore(
            Drive drive, SubsystemScheduler<Superstructure> superstructure, RobotState robotState, Pose2d goalPose) {
        var optionalPose = Optional.of(goalPose);
        return autoScore(drive, superstructure, robotState, () -> optionalPose);
    }

    public static Command autoScore(
            Drive drive,
            SubsystemScheduler<Superstructure> superstructure,
            RobotState robotState,
            Supplier<Optional<Pose2d>> goalPose) {
        var autoScoreState = new AutoScoreState();
        var updateState = runOnce(() -> {
            var scorePose = goalPose.get();
            if (scorePose.isEmpty()) {
                autoScoreState.valid = false;
                return;
            }

            autoScoreState.valid = true;
            autoScoreState.alignPose = scorePose
                    .get()
                    .transformBy(new Transform2d(-alignDistance.get().in(Meters), 0.0, Rotation2d.kZero));
            autoScoreState.bouncePose = scorePose
                    .get()
                    .transformBy(new Transform2d(bounceDistance.get().in(Meters), 0.0, Rotation2d.kZero));
            autoScoreState.reversePose = scorePose
                    .get()
                    .transformBy(new Transform2d(-reverseDistance.get().in(Meters), 0.0, Rotation2d.kZero));
            autoScoreState.releaseDistance = bounceDistance.get().plus(releaseTolerance.get());

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

        Supplier<Command> driveToAlignPose =
                () -> drive.toPose(() -> getTargetPose(robotState.getPose(), autoScoreState.alignPose), false, false);
        var driveToBouncePose =
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoScoreState.bouncePose), false, false);
        var driveToReversePose = drive.toPose(() -> autoScoreState.reversePose, false, false);
        var bounce = sequence(
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoScoreState.bouncePose), true, true),
                drive.toPose(() -> autoScoreState.reversePose, true, true));
        Supplier<Command> stowSuperstructure =
                () -> superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.STOW)));
        Supplier<Command> feedSuperstructure =
                () -> superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.FEED)));
        Supplier<Command> alignSuperstructure =
                () -> superstructure.schedule(s -> s.defer(() -> s.transitionToState(autoScoreState.alignState)));
        var score = superstructure.schedule(
                s -> s.defer(() -> s.score(autoScoreState.alignState, autoScoreState.scoreState, () -> robotState
                        .getDistanceTo(autoScoreState.bouncePose)
                        .lt(autoScoreState.releaseDistance))));

        return sequence(
                updateState,
                sequence(
                                parallel(driveToAlignPose.get(), feedSuperstructure.get())
                                        .until(() -> robotState.hasCoral())
                                        .unless(() -> robotState.hasCoral()),
                                parallel(driveToAlignPose.get(), stowSuperstructure.get())
                                        .until(() -> robotState
                                                .getDistanceTo(autoScoreState.alignPose)
                                                .lt(minArticulationDistance.get())),
                                parallel(driveToAlignPose.get(), alignSuperstructure.get())
                                        .until(() ->
                                                superstructure.getSubsystem().isNear(autoScoreState.alignState)),
                                parallel(driveToBouncePose, alignSuperstructure.get())
                                        .until(() -> robotState
                                                .getDistanceTo(autoScoreState.bouncePose)
                                                .lt(alignDistance.get())),
                                parallel(bounce, score).until(() -> !robotState.hasCoral()),
                                parallel(driveToReversePose, stowSuperstructure.get()))
                        .unless(() -> autoScoreState.valid == false));
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

    private static class AutoScoreState {
        private boolean valid = false;
        private SuperstructureState alignState = SuperstructureState.L4_CORAL;
        private SuperstructureState scoreState = SuperstructureState.L4_CORAL_SCORE;
        private Pose2d alignPose = Pose2d.kZero;
        private Pose2d bouncePose = Pose2d.kZero;
        private Pose2d reversePose = Pose2d.kZero;
        private Distance releaseDistance = Meters.zero();
    }
}
