package frc.robot.autonomous;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralMode;
import frc.robot.autonomous.LoggedAutoRoutine.CoralIntakePose;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
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

    public AutoRoutine rightFour() {
        return create("Right 4")
                .requireStartingPose("Right4")
                .setCoralSelection(CoralMode.L4_CORAL)
                .setSuperstructureState(SuperstructureState.FEED)
                .followPathAndScore("Right4", 0)
                .followPathAndWaitForCoral("Right4", 1)
                .followPathAndScore("Right4", 2)
                .followPathAndWaitForCoral("Right4", 3)
                .followPathAndScore("Right4", 4)
                .followPathAndWaitForCoral("Right4", 5)
                .followPathAndScore("Right4", 6)
                .setSuperstructureState(SuperstructureState.STOW)
                .velocitySeconds(new ChassisSpeeds(), 1)
                .build();
    }

    public AutoRoutine leftFour() {
        return create("Left 4")
                .requireStartingPose("Left4")
                .setCoralSelection(CoralMode.L4_CORAL)
                .setSuperstructureState(SuperstructureState.FEED)
                .followPathAndScore("Left4", 0)
                .followPathAndWaitForCoral("Left4", 1)
                .followPathAndScore("Left4", 2)
                .followPathAndWaitForCoral("Left4", 3)
                .followPathAndScore("Left4", 4)
                .followPathAndWaitForCoral("Left4", 5)
                .followPathAndScore("Left4", 6)
                .setSuperstructureState(SuperstructureState.STOW)
                .velocitySeconds(new ChassisSpeeds(), 1)
                .build();
    }

    public AutoRoutine groundRightFour() {
        return create("Ground Right 4")
                .requireStartingPose("GRight4")
                .setCoralSelection(CoralMode.L4_CORAL)
                .followPathAndScore("GRight4", 0)
                .pickupCoral(CoralIntakePose.RIGHT)
                .setCoralSelection(CoralMode.L2_CORAL)
                .followPathAndScore("GRight4", 2)
                .pickupCoral(CoralIntakePose.CENTER)
                .followPathAndScore("GRight4", 4)
                .pickupCoral(CoralIntakePose.LEFT)
                .setCoralSelection(CoralMode.L4_CORAL)
                .followPathAndScore("GRight4", 6)
                .setSuperstructureState(SuperstructureState.STOW)
                .velocitySeconds(new ChassisSpeeds(), 1)
                .build();
    }

    public AutoRoutine groundRightTwo() {
        return create("Ground Right 2")
                .requireStartingPose("GRight4")
                .setCoralSelection(CoralMode.L4_CORAL)
                .followPathAndScore("GRight4", 0)
                .velocitySeconds(new ChassisSpeeds(-0.3, 0.0, 0.0), 1)
                .waitForSuperstructureState(SuperstructureState.STOW)
                .rotateTo(Rotation2d.k180deg)
                .pickupCoral(CoralIntakePose.CENTER)
                .setSuperstructureState(SuperstructureState.STOW)
                .rotateTo(Rotation2d.kZero)
                .scoreCoral(new Pose2d(3.0, 4.2, Rotation2d.kZero))
                .setSuperstructureState(SuperstructureState.STOW)
                .velocitySeconds(new ChassisSpeeds(), 1)
                .build();
    }

    public AutoRoutine backAlgae() {
        var maxElevatorHeightToTurn = ElevatorConstants.maximumHeight.times(0.75);
        var coralPose = new Pose2d(6.1, 4.1, Rotation2d.k180deg);
        var algae1Pose = new Pose2d(5.8, 4.0, Rotation2d.k180deg);
        var algae2Pose = new Pose2d(5.2, 5.25, Rotation2d.fromDegrees(120));

        return create("Back Algae")
                .setCoralSelection(CoralMode.L4_CORAL)
                .scoreCoral(coralPose)
                .pickupAlgae(algae1Pose)
                .setSuperstructureState(SuperstructureState.L2_ALGAE_RETRACTED)
                .scoreInNet()
                .pointModulesUntil(
                        Rotation2d.k180deg,
                        () -> superstructure.getSubsystem().getElevatorHeight().lt(maxElevatorHeightToTurn))
                .pickupAlgae(algae2Pose)
                .setSuperstructureState(SuperstructureState.L3_ALGAE_RETRACTED)
                .scoreInNet()
                .velocitySeconds(new ChassisSpeeds(-0.5, 0.0, 0.0), 2)
                .build();
    }
}
