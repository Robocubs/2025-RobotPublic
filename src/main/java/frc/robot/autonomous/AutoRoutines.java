package frc.robot.autonomous;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralMode;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoRoutines {
    private final RobotState robotState;
    private final Drive drive;
    private final SubsystemScheduler<Superstructure> superstructure;
    private final AutoFactory factory;

    public AutoRoutines(RobotState robotState, Drive drive, SubsystemScheduler<Superstructure> superstructure) {
        this.robotState = robotState;
        this.drive = drive;
        this.superstructure =
                superstructure; // Intentionally never pass in actual superstructure to avoid multiple commands running
        // at once

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
                                RobotState.isBlue() ? trajectory.getPoses() : GeometryUtil.flip(trajectory.getPoses());
                        Logger.recordOutput("Choreo/Trajectory", poses);
                    } else {
                        Logger.recordOutput("Choreo/Trajectory", new Pose2d[] {});
                        Logger.recordOutput("Choreo/TargetPose", Pose2d.kZero);
                    }
                });

        factory.bind("Feed", runOnce(() -> superstructure.schedule(s -> s.runState(SuperstructureState.FEED))));
        factory.bind("Stow", runOnce(() -> superstructure.schedule(s -> s.runState(SuperstructureState.STOW))));
        factory.bind(
                "Extend",
                superstructure.schedule(s -> defer(
                        () -> s.runState(
                                switch (robotState.getCoralSelection()) {
                                    case L4_CORAL -> SuperstructureState.L4_CORAL;
                                    case L3_CORAL -> SuperstructureState.L3_CORAL;
                                    case L2_CORAL -> SuperstructureState.L2_CORAL;
                                    case L1_CORAL -> SuperstructureState.L1_CORAL;
                                }),
                        Set.of())));
    }

    private LoggedAutoRoutine create(String name) {
        return new LoggedAutoRoutine(name, factory, robotState, drive, superstructure);
    }

    public AutoRoutine demoAuto() {
        return create("Demo Auto")
                .resetPose("SimplePath")
                .followPath("SimplePath", 0)
                .pointModulesWaitSeconds(new Translation2d(8.33, 4), 1.0)
                .driveToPose(new Pose2d(8.33, 4, Rotation2d.kCCW_90deg))
                .build();
    }

    public AutoRoutine forwardAuto() {
        return create("Forward Auto")
                .velocitySeconds(new ChassisSpeeds(1, 0, 0), 5)
                .build();
    }

    public AutoRoutine twoPieceRight() {
        return create("Two Piece Right")
                .setCoralSelection(CoralMode.L4_CORAL)
                .setSuperstructureState(SuperstructureState.FEED)
                .followPathAndScore("TwoPieceRight", 0)
                .followPathAndWaitForCoral("TwoPieceRight", 1)
                .followPathAndScore("TwoPieceRight", 2)
                .build();
    }
}
