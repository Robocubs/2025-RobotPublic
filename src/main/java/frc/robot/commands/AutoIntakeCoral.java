package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public final class AutoIntakeCoral {
    private static final double reefFaceLength = FieldConstants.reefFaceLength.in(Meters);

    public static final LoggedTunableMeasure<DistanceUnit, Distance> maxDistanceReefLineup =
            new LoggedTunableMeasure<>("AutoIntake/MaxDistanceReefLineup", Meters.of(1.0));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> alignDistance =
            new LoggedTunableMeasure<>("AutoIntake/AlignDistance", Meters.of(0.3));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> overshootDistance =
            new LoggedTunableMeasure<>("AutoIntake/OvershootDistance", Meters.of(0.5));
    public static final LoggedTunableMeasure<DistanceUnit, Distance> minArticulationDistance =
            new LoggedTunableMeasure<>("AutoIntake/ArticulationDistance", Meters.of(1.0));
    public static final LoggedTunableMeasure<AngleUnit, Angle> minArticulationAngle =
            new LoggedTunableMeasure<>("AutoIntake/ArticulationAngle", Degrees.of(90));

    public static Command autoIntakeCoral(
            Drive drive, Superstructure superstructure, RobotState robotState, Supplier<Pose2d> goalPose) {
        var scheduler = new SubsystemScheduler<Superstructure>(superstructure, superstructure.maintainState());
        return deadline(autoIntakeCoral(drive, scheduler, robotState, goalPose), scheduler.cmd());
    }

    public static Command autoIntakeCoral(
            Drive drive, SubsystemScheduler<Superstructure> superstructure, RobotState robotState, Pose2d goalPose) {
        return autoIntakeCoral(drive, superstructure, robotState, () -> goalPose);
    }

    public static Command autoIntakeCoral(
            Drive drive,
            SubsystemScheduler<Superstructure> superstructure,
            RobotState robotState,
            Supplier<Pose2d> goalPose) {
        var autoIntakeState = new AutoIntakeState();
        var updateState = runOnce(() -> {
            var goalPoseValue = goalPose.get();

            autoIntakeState.alignPose = goalPoseValue.transformBy(
                    new Transform2d(-alignDistance.get().in(Meters), 0.0, Rotation2d.kZero));
            autoIntakeState.overshootPose = goalPoseValue.transformBy(
                    new Transform2d(overshootDistance.get().in(Meters), 0.0, Rotation2d.kZero));
        });

        Supplier<Command> driveToAlignPose =
                () -> drive.toPose(() -> getTargetPose(robotState.getPose(), autoIntakeState.alignPose), false, false);
        var driveToOvershootPose =
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoIntakeState.overshootPose), true, false);
        var driveToOvershootPoseSlow =
                drive.toPose(() -> getTargetPose(robotState.getPose(), autoIntakeState.overshootPose), true, true);
        Supplier<Command> stowSuperstructure =
                () -> superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.STOW)));
        Supplier<Command> coralIntakeSuperstructure = () ->
                superstructure.schedule(s -> s.defer(() -> s.transitionToState(SuperstructureState.CORAL_INTAKE)));
        Supplier<Command> intake = () -> superstructure.schedule(s -> sequence(
                s.runState(SuperstructureState.CORAL_INTAKE).unless(() -> robotState.hasCoralLoaded()),
                idle().until(() -> robotState.hasCoralLoaded()),
                s.runState(SuperstructureState.STOW)));

        return sequence(
                updateState,
                sequence(
                        parallel(driveToAlignPose.get(), stowSuperstructure.get())
                                .until(() -> GeometryUtil.isNear(
                                                robotState.getHeading(),
                                                autoIntakeState.alignPose.getRotation(),
                                                new Rotation2d(minArticulationAngle.get()))
                                        && robotState
                                                .getDistanceTo(autoIntakeState.alignPose)
                                                .lt(minArticulationDistance.get())),
                        parallel(driveToAlignPose.get(), coralIntakeSuperstructure.get())
                                .until(() -> superstructure.getSubsystem().isNear(SuperstructureState.CORAL_INTAKE))
                                .unless(() -> robotState.hasCoralLoaded()),
                        parallel(driveToOvershootPose, intake.get()).until(() -> robotState
                                .getDistanceTo(autoIntakeState.overshootPose)
                                .lt(alignDistance.get())),
                        parallel(driveToOvershootPoseSlow, intake.get())));
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

    private static class AutoIntakeState {
        private Pose2d alignPose = Pose2d.kZero;
        private Pose2d overshootPose = Pose2d.kZero;
    }
}
