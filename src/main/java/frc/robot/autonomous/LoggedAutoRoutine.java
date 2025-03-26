package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralMode;
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
    private static final LoggedTunableMeasure<DistanceUnit, Distance> scoreDriveToPoseDistance =
            new LoggedTunableMeasure<>("Auto/ScoreDriveToPoseDistance", Meters.of(1.5));

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
        commands.add(drive.toPose(() -> GeometryUtil.autoFlip(pose), true, true));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine pointModulesWaitSeconds(Translation2d target, double seconds) {
        commands.add(Commands.deadline(
                        Commands.waitSeconds(seconds), drive.pointModulesAt(() -> GeometryUtil.autoFlip(target)))
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

        commands.add(AutoScore.autoScore(drive, superstructure, robotState, goalPose));
        pathBuilder.add(GeometryUtil.autoFlip(goalPose));

        commands.add(superstructure.schedule(s -> s.runState(SuperstructureState.FEED)));

        return this;
    }

    public LoggedAutoRoutine waitForCoral(Pose2d pose) {
        commands.add(parallel(
                        drive.toPose(() -> GeometryUtil.autoFlip(pose), false, true),
                        superstructure.schedule(s -> s.runState(SuperstructureState.FEED)))
                .until(robotState::hasCoral)
                .withTimeout(1.0));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine followPathAndWaitForCoral(String pathName, int splitIndex) {
        var path = routine.trajectory(pathName, splitIndex);
        commands.add(path.cmd());

        var poses = path.getRawTrajectory().getPoses();
        pathBuilder.add(poses);

        path.getRawTrajectory().getFinalPose(false).ifPresent(this::waitForCoral);

        return this;
    }

    public LoggedAutoRoutine followPathAndScore(String pathName, int splitIndex) {
        var path = routine.trajectory(pathName, splitIndex);
        var finalPose = path.getRawTrajectory().getFinalPose(false);
        commands.add(path.cmd()
                .until(() -> finalPose.isPresent()
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
}
