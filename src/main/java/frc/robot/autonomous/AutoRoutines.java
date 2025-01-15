package frc.robot.autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeometryUtil;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
    private final RobotState robotState;
    private final Drive drive;
    private final AutoFactory factory;

    public AutoRoutines(RobotState robotState, Drive drive) {
        this.robotState = robotState;
        this.drive = drive;

        factory = new AutoFactory(
                robotState::getPose,
                robotState::resetPose,
                drive::followPath,
                true,
                drive,
                // autoBindings,
                (trajectory, starting) -> {
                    if (starting) {
                        var poses =
                                RobotState.isBlue() ? trajectory.getPoses() : GeometryUtil.flipX(trajectory.getPoses());
                        Logger.recordOutput("Choreo/Trajectory", poses);
                    } else {
                        Logger.recordOutput("Choreo/Trajectory", new Pose2d[] {});
                        Logger.recordOutput("Choreo/TargetPose", Pose2d.kZero);
                    }
                });
    }

    private LoggedAutoRoutine create(String name) {
        return new LoggedAutoRoutine(name, factory, robotState, drive);
    }

    public AutoRoutine demoAuto() {
        return create("Demo Auto")
                .resetPose("SimplePath")
                .followPath("SimplePath")
                .pointModulesWaitSeconds(new Translation2d(8.33, 4), 1.0)
                .driveToPose(new Pose2d(8.33, 4, Rotation2d.kCCW_90deg))
                .build();
    }

    public AutoRoutine forwardAuto() {
        return create("Forward Auto")
                .velocitySeconds(new ChassisSpeeds(1, 0, 0), 5)
                .build();
    }
}
