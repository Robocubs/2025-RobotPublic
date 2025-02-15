package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.logging.LoggedCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeometryUtil;
import frc.robot.util.PathBuilder;
import org.littletonrobotics.junction.Logger;

public class LoggedAutoRoutine {
    private final AutoFactory factory;
    private final AutoRoutine routine;
    private final String name;
    private final RobotState robotState;
    private final Drive drive;
    private final List<Command> commands = new ArrayList<>();
    private final PathBuilder pathBuilder = new PathBuilder();

    private boolean bound = false;

    public LoggedAutoRoutine(String name, AutoFactory factory, RobotState robotState, Drive drive) {
        this.factory = factory;
        this.routine = factory.newRoutine(name);
        this.name = name;
        this.robotState = robotState;
        this.drive = drive;
    }

    public AutoRoutine build() {
        if (!bound) {
            routine.active()
                    .onTrue(Commands.sequence(
                            Commands.print("Starting " + name),
                            LoggedCommands.loggedSequence(commands.toArray(Command[]::new)),
                            Commands.print("Finished " + name)));
        }

        Logger.recordOutput("Auto/Path", GeometryUtil.autoFlip(pathBuilder.build()));

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
}
