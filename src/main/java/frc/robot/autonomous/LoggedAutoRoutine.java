package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralMode;
import frc.robot.commands.AutoIntakeAlgae;
import frc.robot.commands.AutoIntakeCoral;
import frc.robot.commands.AutoScore;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.commands.logging.LoggedCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import frc.robot.util.PathBuilder;
import frc.robot.util.tuning.LoggedTunableMeasure;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class LoggedAutoRoutine {
    private static final Pose2d netAlignPose = new Pose2d(7, 4.9, Rotation2d.kZero);
    private static final Pose2d netScorePose = new Pose2d(7.5, 4.9, Rotation2d.kZero);

    private static final LoggedTunableMeasure<DistanceUnit, Distance> scoreDriveToPoseDistance =
            new LoggedTunableMeasure<>("Auto/ScoreDriveToPoseDistance", Meters.of(1.5));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> pickupCoralArticulationDistance =
            new LoggedTunableMeasure<>("Auto/PickupCoralArticulationDistance", Meters.of(0.3));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> scoreAlgaeDistance =
            new LoggedTunableMeasure<>("Auto/ScoreAlgaeDistance", Meters.of(0.05));

    private final AutoFactory factory;
    private final AutoRoutine routine;
    private final String name;
    private final RobotState robotState;
    private final Drive drive;
    private final SubsystemScheduler<Superstructure> superstructure;
    private final List<Command> commands = new ArrayList<>();
    private final PathBuilder pathBuilder = new PathBuilder();

    private boolean bound = false;

    public LoggedAutoRoutine(
            String name,
            AutoFactory factory,
            RobotState robotState,
            Drive drive,
            SubsystemScheduler<Superstructure> superstructureScheduler) {
        this.factory = factory;
        this.routine = factory.newRoutine(name);
        this.name = name;
        this.robotState = robotState;
        this.drive = drive;
        this.superstructure = superstructureScheduler;
    }

    public AutoRoutine build() {
        if (!bound) {
            routine.active()
                    .onTrue(sequence(
                            print("Starting " + name),
                            parallel(
                                    superstructure.cmd(),
                                    LoggedCommands.loggedSequence(commands.toArray(Command[]::new))),
                            print("Finished " + name)));
        }

        var path = GeometryUtil.autoFlip(pathBuilder.build());
        Logger.recordOutput("Auto/Path", path);
        robotState.setPath(path);

        return routine;
    }

    public LoggedAutoRoutine requireStartingPose(String pathName) {
        if (Constants.mode == Constants.Mode.SIM) {
            return this.resetPose(pathName);
        }

        var trajectory = routine.trajectory(pathName);
        if (trajectory == null) {
            commands.add(runOnce(() -> System.out.println("Trajectory not found, doing nothing"))
                    .andThen(idle()));
            return this;
        }

        var startingPoseOptional = trajectory.getRawTrajectory().getInitialPose(false);
        if (startingPoseOptional.isEmpty()) {
            commands.add(runOnce(() -> System.out.println("Starting pose not found, doing nothing"))
                    .andThen(idle()));
            return this;
        }

        var startingPose = startingPoseOptional.get();
        commands.add(print("Starting pose too far, doing nothing")
                .andThen(idle())
                .unless(() -> robotState
                                .getPose()
                                .getTranslation()
                                .getDistance(GeometryUtil.autoFlip(startingPose.getTranslation()))
                        < 1.0));
        return this;
    }

    public LoggedAutoRoutine run(Command command) {
        return this;
    }

    public LoggedAutoRoutine waitSeconds(double seconds) {
        commands.add(Commands.waitSeconds(seconds));
        return this;
    }

    public LoggedAutoRoutine velocitySeconds(ChassisSpeeds chassisSpeeds, double seconds) {
        commands.add(drive.withSpeeds(() -> chassisSpeeds).withTimeout(seconds));
        return this;
    }

    public LoggedAutoRoutine rotateTo(Rotation2d rotation) {
        commands.add(drive.toPose(
                () -> {
                    var pose = robotState.getPose();
                    return new Pose2d(pose.getX(), pose.getY(), rotation);
                },
                true,
                false));
        return this;
    }

    public LoggedAutoRoutine resetRotation(Rotation2d rotation) {
        commands.add(runOnce(() -> {
            var pose = robotState.getPose();
            robotState.resetPose(new Pose2d(pose.getX(), pose.getY(), GeometryUtil.autoFlip(rotation)));
        }));

        return this;
    }

    public LoggedAutoRoutine resetPose(String pathName) {
        commands.add(factory.resetOdometry(pathName));
        routine.trajectory(pathName).getRawTrajectory().getInitialPose(false).ifPresent(pathBuilder::add);
        return this;
    }

    public LoggedAutoRoutine followPath(String pathName) {
        var path = routine.trajectory(pathName);
        commands.add(path.cmd());
        pathBuilder.add(path.getRawTrajectory().getPoses());
        return this;
    }

    public LoggedAutoRoutine followPath(String pathName, int splitIndex) {
        var path = routine.trajectory(pathName, splitIndex);
        commands.add(path.cmd());
        pathBuilder.add(path.getRawTrajectory().getPoses());
        return this;
    }

    public LoggedAutoRoutine driveToPose(Pose2d pose) {
        driveToPose(pose, false);
        return this;
    }

    public LoggedAutoRoutine driveToPose(Pose2d pose, boolean slowMode) {
        commands.add(drive.toPose(() -> GeometryUtil.autoFlip(pose), true, slowMode));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine pointModulesUntil(Rotation2d target, BooleanSupplier condition) {
        commands.add(drive.pointModules(() -> target).until(condition).withName("PointModulesAndWaitForCondition"));
        return this;
    }

    public LoggedAutoRoutine pointModulesUntil(Translation2d target, BooleanSupplier condition) {
        commands.add(drive.pointModulesAt(() -> GeometryUtil.autoFlip(target))
                .until(condition)
                .withName("PointModulesAndWaitForCondition"));
        return this;
    }

    public LoggedAutoRoutine pointModulesWaitSeconds(Translation2d target, double seconds) {
        commands.add(drive.pointModulesAt(() -> GeometryUtil.autoFlip(target))
                .withTimeout(seconds)
                .withName("PointModulesAndWait"));
        return this;
    }

    public LoggedAutoRoutine setCoralSelection(CoralMode mode) {
        commands.add(runOnce(() -> robotState.setCoralSelection(mode)));
        return this;
    }

    public LoggedAutoRoutine setSuperstructureState(SuperstructureState state) {
        commands.add(superstructure.schedule(s -> s.runState(state)));
        return this;
    }

    public LoggedAutoRoutine scoreCoral(Pose2d pose) {
        var flippedPose = GeometryUtil.autoFlip(pose);
        var goalPose = robotState.getClosestReefBranch(flippedPose).orElse(flippedPose);

        commands.add(AutoScore.autoScore(drive, superstructure, robotState, goalPose)
                .until(() -> !robotState.hasCoralLoaded())
                .unless(() -> !robotState.hasCoralLoaded()));
        pathBuilder.add(GeometryUtil.autoFlip(goalPose));

        return this;
    }

    public LoggedAutoRoutine waitForCoral(Pose2d pose) {
        commands.add(sequence(
                        superstructure.schedule(s -> s.runState(SuperstructureState.FEED)),
                        drive.toPose(() -> GeometryUtil.autoFlip(pose), true, true),
                        drive.pointModules(() -> Rotation2d.kZero).withTimeout(0.5))
                .until(robotState::hasCoralLoaded));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine waitForSuperstructureState(SuperstructureState state) {
        commands.add(Commands.parallel(
                        drive.pointModules(() -> Rotation2d.kZero), superstructure.schedule(s -> s.runState(state)))
                .until(() -> superstructure.getSubsystem().isNear(state)));
        return this;
    }

    public LoggedAutoRoutine followPathAndWaitForCoral(String pathName, int splitIndex) {
        commands.add(superstructure.schedule(s -> s.runState(SuperstructureState.FEED)));

        var path = routine.trajectory(pathName, splitIndex);
        commands.add(path.cmd());

        var poses = path.getRawTrajectory().getPoses();
        pathBuilder.add(poses);

        path.getRawTrajectory().getFinalPose(false).ifPresent(this::waitForCoral);

        return this;
    }

    public LoggedAutoRoutine followPathAndPickupCoral(String pathName, int splitIndex) {
        var path = routine.trajectory(pathName, splitIndex);
        var initialPose = path.getRawTrajectory().getInitialPose(false);
        commands.add(path.cmd()
                .deadlineFor(sequence(
                        superstructure.schedule(s -> s.runState(SuperstructureState.STOW)),
                        idle().until(() -> initialPose.isPresent()
                                && robotState
                                        .getDistanceTo(GeometryUtil.autoFlip(initialPose.get()))
                                        .gt(pickupCoralArticulationDistance.get())),
                        superstructure.schedule(s -> s.runState(SuperstructureState.CORAL_INTAKE)),
                        idle().until(() -> robotState.hasCoralLoaded()),
                        superstructure.schedule(s -> s.runState(SuperstructureState.STOW)))));

        var poses = path.getRawTrajectory().getPoses();
        pathBuilder.add(poses);

        this.setSuperstructureState(SuperstructureState.STOW);

        return this;
    }

    public LoggedAutoRoutine pickupCoral(CoralIntakePose pose) {
        commands.add(
                AutoIntakeCoral.autoIntakeCoral(drive, superstructure, robotState, GeometryUtil.autoFlip(pose.pose)));
        pathBuilder.add(pose.pose);
        commands.add(superstructure.schedule(s -> s.runState(SuperstructureState.STOW)));
        return this;
    }

    public LoggedAutoRoutine pickupAlgae(String pathName, int splitIndex) {
        routine.trajectory(pathName, splitIndex)
                .getRawTrajectory()
                .getFinalPose(false)
                .ifPresent(pose -> pickupAlgae(pose));
        return this;
    }

    public LoggedAutoRoutine pickupAlgae(Pose2d pose) {
        commands.add(AutoIntakeAlgae.autoIntakeAlgae(drive, superstructure, robotState, GeometryUtil.autoFlip(pose)));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine followPathAndScore(String pathName, int splitIndex) {
        var path = routine.trajectory(pathName, splitIndex);
        var finalPose = path.getRawTrajectory().getFinalPose(false);
        commands.add(path.cmd()
                .until(() -> robotState.hasCoralLoaded()
                        && finalPose.isPresent()
                        && robotState
                                .getDistanceTo(GeometryUtil.autoFlip(finalPose.get()))
                                .lt(scoreDriveToPoseDistance.get())));

        var poses = path.getRawTrajectory().getPoses();
        pathBuilder.add(poses);

        path.getRawTrajectory()
                .getFinalPose(false)
                .ifPresent(pose -> this.scoreCoral(new Pose2d(pose.getTranslation(), pose.getRotation())));

        return this;
    }

    public LoggedAutoRoutine scoreInNet() {
        driveToPose(netAlignPose);

        var score = deadline(
                sequence(
                        idle().until(() -> robotState
                                .getDistanceTo(GeometryUtil.autoFlip(netScorePose))
                                .lt(scoreAlgaeDistance.get())),
                        superstructure.schedule(s -> s.runState(SuperstructureState.BARGE_SCORE)),
                        waitTime(Seconds.of(1.0))),
                drive.toPose(() -> GeometryUtil.autoFlip(netScorePose), false, true));

        commands.add(sequence(
                        superstructure.schedule(s -> s.runState(SuperstructureState.BARGE)),
                        drive.pointModules(() -> Rotation2d.kZero)
                                .until(() -> superstructure.getSubsystem().elevatorIsNear(SuperstructureState.BARGE)),
                        score,
                        superstructure.schedule(s -> s.runState(SuperstructureState.STOW)))
                .withName("AutoScoreInNet"));
        pathBuilder.add(netScorePose);

        return this;
    }

    public static enum CoralIntakePose {
        LEFT(new Pose2d(1.5, 5.4, Rotation2d.fromDegrees(120))),
        CENTER(new Pose2d(1.8, 4.04, Rotation2d.k180deg)),
        RIGHT(new Pose2d(1.5, 2.67, Rotation2d.fromDegrees(-120)));

        private final Pose2d pose;

        private CoralIntakePose(Pose2d pose) {
            this.pose = pose;
        }
    }
}
