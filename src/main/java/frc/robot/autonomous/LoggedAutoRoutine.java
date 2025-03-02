package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralMode;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.commands.logging.LoggedCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import frc.robot.util.PathBuilder;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class LoggedAutoRoutine {
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
        commands.add(drive.toPose(() -> GeometryUtil.autoFlip(pose), true));
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
        var debouncer = new Debouncer(0.1, DebounceType.kRising);
        var driveToPose = drive.toPose(() -> goalPose, false);
        var score = superstructure.schedule(s -> Commands.defer(
                () -> s.score(
                        switch (robotState.getCoralSelection()) {
                            case L4_CORAL -> SuperstructureState.L4_CORAL;
                            case L3_CORAL -> SuperstructureState.L3_CORAL;
                            case L2_CORAL -> SuperstructureState.L2_CORAL;
                            case L1_CORAL -> SuperstructureState.L1_CORAL;
                        },
                        switch (robotState.getCoralSelection()) {
                            case L4_CORAL -> SuperstructureState.L4_CORAL_SCORE;
                            case L3_CORAL -> SuperstructureState.L3_CORAL_SCORE;
                            case L2_CORAL -> SuperstructureState.L2_CORAL_SCORE;
                            case L1_CORAL -> SuperstructureState.L1_CORAL_SCORE;
                        },
                        () -> debouncer.calculate(driveToPose.atGoal())),
                Set.of()));
        commands.add(sequence(
                runOnce(() -> debouncer.calculate(false)),
                parallel(driveToPose, score).until(() -> !robotState.hasLongCoral()),
                superstructure.schedule(s -> s.runState(SuperstructureState.STOW))));
        pathBuilder.add(goalPose);
        return this;
    }

    public LoggedAutoRoutine waitForCoral(Pose2d pose) {
        commands.add(parallel(
                        drive.toPose(() -> GeometryUtil.autoFlip(pose), false),
                        superstructure.schedule(s -> s.runState(SuperstructureState.FEED)))
                .until(robotState::hasLongCoral));
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
        commands.add(path.cmd());

        var poses = path.getRawTrajectory().getPoses();
        pathBuilder.add(poses);

        path.getRawTrajectory()
                .getFinalPose(false)
                .ifPresent(pose -> this.scoreCoral(new Pose2d(pose.getTranslation(), pose.getRotation())));

        return this;
    }
}
