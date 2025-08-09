package frc.robot.commands;

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
import frc.robot.RobotState.AlgaeMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public final class AutoIntakeAlgae {
    private static final double reefFaceLength = FieldConstants.reefFaceLength.in(Meters);

    public static final LoggedTunableMeasure<DistanceUnit, Distance> maxDistanceReefLineup =
            new LoggedTunableMeasure<>("AutoIntakeAlgae/MaxDistanceReefLineup", Meters.of(1.0));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> alignDistance =
            new LoggedTunableMeasure<>("AutoIntakeAlgae/AlignDistance", Meters.of(0.3));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> overshootDistance =
            new LoggedTunableMeasure<>("AutoIntakeAlgae/OvershootDistance", Meters.of(0.1));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> reverseDistance =
            new LoggedTunableMeasure<>("AutoIntakeAlgae/ReverseDistance", Meters.of(0.5));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> minArticulationDistance =
            new LoggedTunableMeasure<>("AutoIntakeAlgae/ArticulationDistance", Meters.of(1.0));

    public static Command intakeHighAlgae(
            Drive drive, Superstructure superstructure, RobotState robotState, Supplier<Pose2d> goalPose) {
        var scheduler = new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        return deadline(autoIntakeAlgae(drive, scheduler, robotState, goalPose, AlgaeLevel.L3), scheduler.cmd());
    }

    public static Command intakeLowAlgae(
            Drive drive, Superstructure superstructure, RobotState robotState, Supplier<Pose2d> goalPose) {
        var scheduler = new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        return deadline(autoIntakeAlgae(drive, scheduler, robotState, goalPose, AlgaeLevel.L2), scheduler.cmd());
    }

    public static Command autoIntakeAlgae(
            Drive drive, Superstructure superstructure, RobotState robotState, Supplier<Pose2d> goalPose) {
        var scheduler = new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        return deadline(autoIntakeAlgae(drive, scheduler, robotState, goalPose, AlgaeLevel.AUTO), scheduler.cmd());
    }

    public static Command autoIntakeAlgae(
            Drive drive, SubsystemScheduler<Superstructure> superstructure, RobotState robotState, Pose2d goalPose) {
        return autoIntakeAlgae(drive, superstructure, robotState, () -> goalPose, AlgaeLevel.AUTO);
    }

    public static Command autoIntakeAlgae(
            Drive drive,
            SubsystemScheduler<Superstructure> superstructure,
            RobotState robotState,
            Supplier<Pose2d> goalPose,
            AlgaeLevel level) {
        var autoIntakeState = new AutoIntakeAlgaeState();
        var updateState = runOnce(() -> {
            var algaePose = robotState.getClosestReefAlgae(goalPose.get());
            if (algaePose.isEmpty()) {
                autoIntakeState.valid = false;
                return;
            }

            autoIntakeState.valid = true;
            switch (level) {
                case L3:
                    autoIntakeState.intakeState = SuperstructureState.L3_ALGAE;
                    autoIntakeState.removalState = SuperstructureState.L3_ALGAE_REMOVAL;
                    break;
                case L2:
                    autoIntakeState.intakeState = SuperstructureState.L2_ALGAE;
                    autoIntakeState.intakeState = SuperstructureState.L2_ALGAE_REMOVAL;
                    break;
                case AUTO:
                default:
                    if (robotState.getReefAlgaeMode(algaePose.get().getRotation()) == AlgaeMode.L2) {
                        autoIntakeState.intakeState = SuperstructureState.L2_ALGAE;
                        autoIntakeState.removalState = SuperstructureState.L2_ALGAE_REMOVAL;
                    } else {
                        autoIntakeState.intakeState = SuperstructureState.L3_ALGAE;
                        autoIntakeState.removalState = SuperstructureState.L3_ALGAE_REMOVAL;
                    }
                    break;
            }
            autoIntakeState.alignPose = algaePose
                    .get()
                    .transformBy(new Transform2d(-alignDistance.get().in(Meters), 0.0, Rotation2d.kZero));
            autoIntakeState.overshootPose = algaePose
                    .get()
                    .transformBy(new Transform2d(overshootDistance.get().in(Meters), 0.0, Rotation2d.kZero));
            autoIntakeState.finalPose = algaePose
                    .get()
                    .transformBy(new Transform2d(-reverseDistance.get().in(Meters), 0.0, Rotation2d.kZero));
        });

        Supplier<Command> driveToAlignPose =
                () -> drive.toPose(() -> getTargetPose(robotState.getPose(), autoIntakeState.alignPose), false, false);
        var driveToOvershootPose =
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoIntakeState.overshootPose), false, false);
        var driveToOvershootPoseSlow =
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoIntakeState.overshootPose), false, true);
        var driveToFinalPose = drive.toPose(() -> autoIntakeState.finalPose, true, false);
        var stowSuperstructure =
                superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.STOW)));
        var algaeIntakeSuperstructure =
                superstructure.schedule(s -> s.defer(() -> s.transitionToState(autoIntakeState.intakeState)));
        var algaeRemovalSuperstructure =
                superstructure.schedule(s -> s.defer(() -> s.transitionToState(autoIntakeState.removalState)));

        return sequence(
                updateState,
                sequence(
                                stowSuperstructure.unless(() -> robotState
                                        .getDistanceTo(autoIntakeState.alignPose)
                                        .lt(minArticulationDistance.get())),
                                driveToAlignPose.get().until(() -> robotState
                                        .getDistanceTo(autoIntakeState.alignPose)
                                        .lt(minArticulationDistance.get())),
                                algaeIntakeSuperstructure,
                                driveToAlignPose.get().until(() -> superstructure
                                        .getSubsystem()
                                        .elevatorIsNear(autoIntakeState.intakeState)),
                                driveToOvershootPose.until(() -> robotState
                                        .getDistanceTo(autoIntakeState.overshootPose)
                                        .lt(alignDistance.get())),
                                driveToOvershootPoseSlow.until(() -> robotState.hasAlgae()),
                                algaeRemovalSuperstructure,
                                driveToFinalPose)
                        .unless(() -> !autoIntakeState.valid));
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

    private static class AutoIntakeAlgaeState {
        private boolean valid = false;
        private Pose2d alignPose = Pose2d.kZero;
        private Pose2d overshootPose = Pose2d.kZero;
        private Pose2d finalPose = Pose2d.kZero;
        private SuperstructureState intakeState = SuperstructureState.L2_ALGAE;
        private SuperstructureState removalState = SuperstructureState.L2_ALGAE_REMOVAL;
    }

    private enum AlgaeLevel {
        AUTO,
        L3,
        L2
    }
}
