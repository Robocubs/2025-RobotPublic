package frc.robot;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveMeasurement;
import frc.robot.subsystems.superstructure.SuperstructurePose;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.GeometryUtil;
import frc.robot.util.tuning.LoggedTunableBoolean;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    public static final LoggedTunableBoolean automaticAlgaeModeSelection =
            new LoggedTunableBoolean("RobotState/AutomaticAlgaeModeSelection", true);

    private static final Rotation2d reefPoseAngleTolerance = Rotation2d.fromDegrees(29.9);
    private static final Rotation2d facingProcessorTolerance = Rotation2d.fromDegrees(45.0);
    private static final Rotation2d facingBargeTolerance = Rotation2d.fromDegrees(45.0);
    private static final Distance reefAreaDistanceTolerance = Meters.of(2.5);
    private static final Distance processorAreaDistanceTolerance = Meters.of(2.0);
    private static final Distance bargeAreaLengthTolerance = Meters.of(2.0);
    private static final Distance underNetAreaTolerance = Meters.of(1.3);
    private static final Distance speedLimitMinHeight = Meters.of(0.5);
    private static final LinearVelocity nominalSpeedTolerance = MetersPerSecond.of(0.1);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            Rotation2d.kZero,
            Stream.generate(SwerveModulePosition::new).limit(4).toArray(SwerveModulePosition[]::new),
            Pose2d.kZero,
            DriveConstants.odometryStdDevs,
            VecBuilder.fill(1, 1, 1) // Vision std devs will be passed in with vision measurements
            );
    private final Field2d field;

    private @AutoLogOutput @Getter CoralMode coralSelection = CoralMode.L4_CORAL;
    private @AutoLogOutput @Getter AlgaeMode algaeSelection = AlgaeMode.NONE;
    private @AutoLogOutput @Getter ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter ChassisSpeeds fieldVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter LinearVelocity robotSpeed = MetersPerSecond.zero();
    private @AutoLogOutput @Getter LinearVelocity maxSpeed = DriveConstants.maxSpeed;
    private @Getter Rotation2d headingToReef = Rotation2d.kZero;
    private @AutoLogOutput boolean isFacingReef;
    private @AutoLogOutput boolean isFacingProcessor;
    private @AutoLogOutput boolean isFacingBarge;
    private @AutoLogOutput boolean inReefArea;
    private @AutoLogOutput boolean inProcessorArea;
    private @AutoLogOutput boolean inBargeArea;
    private @AutoLogOutput boolean underNetArea;
    private Distance elevatorheight = Meters.zero();
    private boolean hasLongCoral;
    private boolean hasWideCoral;
    private boolean hasAlgae;

    public static enum CoralMode {
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL
    }

    public static enum AlgaeMode {
        NONE,
        PROCESSOR,
        L2,
        L3,
        BARGE
    }

    public RobotState() {
        field = new Field2d();

        var periodic = Commands.run(this::periodic).ignoringDisable(true).withName("RobotStatePeriodic");
        periodic.schedule();
        RobotModeTriggers.test().onFalse(periodic);

        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("IsSimulation", Robot.isSimulation());
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static boolean isRed() {
        return !isBlue();
    }

    public void periodic() {
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        var pose = poseEstimator.getEstimatedPosition();
        var translation = pose.getTranslation();

        headingToReef =
                FieldConstants.reefCenterPoint().minus(pose.getTranslation()).getAngle();

        isFacingReef = GeometryUtil.isNear(headingToReef, getHeading(), Rotation2d.kCCW_90deg);

        isFacingProcessor = GeometryUtil.isNear(
                getHeading(), FieldConstants.robotProcessorPose().getRotation(), facingProcessorTolerance);

        isFacingBarge = GeometryUtil.isNear(
                pose.getX() < FieldConstants.fieldCenter.getX() ? Rotation2d.kZero : Rotation2d.k180deg,
                getHeading(),
                facingBargeTolerance);

        inReefArea = FieldConstants.reefCenterPoint().getDistance(pose.getTranslation())
                < reefAreaDistanceTolerance.in(Meters);

        inProcessorArea = FieldConstants.processorPose().getTranslation().getDistance(pose.getTranslation())
                < processorAreaDistanceTolerance.in(Meters);

        inBargeArea = MathUtil.isNear(
                        translation.getX(), FieldConstants.fieldCenter.getX(), bargeAreaLengthTolerance.in(Meters))
                && (isBlue()
                        ? translation.getY() > FieldConstants.fieldCenter.getY()
                        : translation.getY() < FieldConstants.fieldCenter.getY());

        underNetArea =
                MathUtil.isNear(translation.getX(), FieldConstants.fieldCenter.getX(), underNetAreaTolerance.in(Meters))
                        && (isBlue()
                                ? translation.getY() > FieldConstants.fieldCenter.getY()
                                : translation.getY() < FieldConstants.fieldCenter.getY());

        if (automaticAlgaeModeSelection.get()) {
            if (inReefArea && isFacingReef) {
                algaeSelection = MathUtil.inputModulus(pose.getRotation().getDegrees() + 30, 0, 360) % 120 > 60
                        ? (isBlue() ? AlgaeMode.L2 : AlgaeMode.L3)
                        : (isBlue() ? AlgaeMode.L3 : AlgaeMode.L2);
            } else if (inProcessorArea && isFacingProcessor) {
                algaeSelection = AlgaeMode.PROCESSOR;
            } else if (inBargeArea && isFacingBarge) {
                algaeSelection = AlgaeMode.BARGE;
            } else {
                algaeSelection = AlgaeMode.NONE;
            }
        }
    }

    @AutoLogOutput
    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(poseEstimator.getEstimatedPosition());
    }

    public Optional<Pose2d> getPose(double timestamp) {
        return poseEstimator.sampleAt(timestamp);
    }

    @AutoLogOutput
    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public boolean isFacingReef() {
        return isFacingReef;
    }

    public boolean isFacingProcessor() {
        return isFacingProcessor;
    }

    public boolean isFacingBarge() {
        return isFacingBarge;
    }

    public boolean inReefArea() {
        return inReefArea;
    }

    public boolean inProcessorArea() {
        return inProcessorArea;
    }

    public boolean inBargeArea() {
        return inBargeArea;
    }

    public boolean underNetArea() {
        return underNetArea;
    }

    public boolean robotSpeedNominal(SuperstructureState state) {
        return robotSpeedNominal(state.getData().getPose());
    }

    public boolean robotSpeedNominal(SuperstructurePose pose) {
        return robotSpeedNominal(pose.elevatorHeight());
    }

    public boolean robotSpeedNominal(Distance elevatorHeight) {
        boolean speedNominal;

        if (elevatorheight.lte(this.elevatorheight)) {
            // Always allow downward movements
            speedNominal = true;
        } else {
            // Otherwise, speed must be below max speed for pose
            var maxSpeed = getMaxRobotSpeed(elevatorHeight);
            speedNominal = robotSpeed.lt(maxSpeed) || robotSpeed.isNear(maxSpeed, nominalSpeedTolerance);
        }

        Logger.recordOutput("RobotState/SpeedNominal", speedNominal);
        return speedNominal;
    }

    public boolean hasLongCoral() {
        return hasLongCoral;
    }

    public boolean hasWideCoral() {
        return hasWideCoral;
    }

    public boolean hasAlgae() {
        return hasAlgae;
    }

    public boolean hasGamePiece() {
        return hasLongCoral || hasWideCoral || hasAlgae;
    }

    public Optional<Pose2d> getClosestReefBranch() {
        return getClosestReefBranch(poseEstimator.getEstimatedPosition());
    }

    public Optional<Pose2d> getClosestReefBranch(Pose2d robotPose) {
        return getClosestReefPose(
                robotPose,
                switch (coralSelection) {
                    case L4_CORAL -> FieldConstants.robotCoralL4Poses();
                    case L3_CORAL -> FieldConstants.robotCoralL3Poses();
                    case L2_CORAL -> FieldConstants.robotCoralL2Poses();
                    case L1_CORAL -> FieldConstants.robotCoralL1Poses();
                });
    }

    public Optional<Pose2d> getClosestReefAlgae() {
        return getClosestReefAlgae(poseEstimator.getEstimatedPosition());
    }

    public Optional<Pose2d> getClosestReefAlgae(Pose2d robotPose) {
        return getClosestReefPose(robotPose, FieldConstants.robotReefAlgaePoses());
    }

    public Optional<Pose2d> getClosestL2ReefAlgae() {
        return getClosestReefAlgae(poseEstimator.getEstimatedPosition());
    }

    public Optional<Pose2d> getClosestL2ReefAlgae(Pose2d robotPose) {
        return getClosestReefPose(robotPose, FieldConstants.robotReefAlgaeL2Poses());
    }

    public Optional<Pose2d> getClosestL3ReefAlgae() {
        return getClosestReefAlgae(poseEstimator.getEstimatedPosition());
    }

    public Optional<Pose2d> getClosestL3ReefAlgae(Pose2d robotPose) {
        return getClosestReefPose(robotPose, FieldConstants.robotReefAlgaeL3Poses());
    }

    private Optional<Pose2d> getClosestReefPose(Pose2d robotPose, Pose2d[] reefPoses) {
        Pose2d closestPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (var reefPose : reefPoses) {
            if (!GeometryUtil.isNear(robotPose.getRotation(), reefPose.getRotation(), reefPoseAngleTolerance)) {
                continue;
            }

            var distance = robotPose.getTranslation().getDistance(reefPose.getTranslation());
            if (distance < closestDistance) {
                closestPose = reefPose;
                closestDistance = distance;
            }
        }

        return Optional.ofNullable(closestPose);
    }

    public Optional<Pose2d> getLeftReefPose() {
        return getLeftReefPose(poseEstimator.getEstimatedPosition().getRotation());
    }

    public Optional<Pose2d> getRightReefPose() {
        return getRightReefPose(poseEstimator.getEstimatedPosition().getRotation());
    }

    public Optional<Pose2d> getLeftReefPose(Rotation2d robotRotation) {
        return getReefPose(
                robotRotation,
                true,
                switch (coralSelection) {
                    case L4_CORAL -> FieldConstants.robotCoralL4Poses();
                    case L3_CORAL -> FieldConstants.robotCoralL3Poses();
                    case L2_CORAL -> FieldConstants.robotCoralL2Poses();
                    case L1_CORAL -> FieldConstants.robotCoralL1Poses();
                });
    }

    public Optional<Pose2d> getRightReefPose(Rotation2d robotRotation) {
        return getReefPose(
                robotRotation,
                false,
                switch (coralSelection) {
                    case L4_CORAL -> FieldConstants.robotCoralL4Poses();
                    case L3_CORAL -> FieldConstants.robotCoralL3Poses();
                    case L2_CORAL -> FieldConstants.robotCoralL2Poses();
                    case L1_CORAL -> FieldConstants.robotCoralL1Poses();
                });
    }

    private Optional<Pose2d> getReefPose(Rotation2d robotRotation, boolean left, Pose2d[] reefPoses) {
        Pose2d pose = null;
        var wantsLargest = left && isBlue() || !left && isRed();
        for (var reefPose : reefPoses) {
            if (!GeometryUtil.isNear(robotRotation, reefPose.getRotation(), reefPoseAngleTolerance)) {
                continue;
            }

            if (pose == null
                    || (wantsLargest && reefPose.getY() > pose.getY())
                    || (!wantsLargest && reefPose.getY() < pose.getY())) {
                pose = reefPose;
            }
        }

        return Optional.ofNullable(pose);
    }

    public void addDriveMeasurements(DriveMeasurement... measurements) {
        if (measurements.length == 0) {
            return;
        }

        for (var measurement : measurements) {
            poseEstimator.updateWithTime(
                    measurement.timestampSeconds, measurement.gyroAngle, measurement.modulePositions);
        }

        robotVelocity = measurements[measurements.length - 1].chassisSpeeds;
        fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getHeading());
        robotSpeed = MetersPerSecond.of(Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));
    }

    public void addVisionMeasurements(VisionMeasurement... measurements) {
        for (var measurement : measurements) {
            poseEstimator.setVisionMeasurementStdDevs(measurement.stdDevs);
            poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestampSeconds);
        }
    }

    public void setGamePieceStates(boolean hasLongCoral, boolean hasWideCoral, boolean hasAlgae) {
        this.hasLongCoral = hasLongCoral;
        this.hasWideCoral = hasWideCoral;
        this.hasAlgae = hasAlgae;
    }

    public void setPath(Pose2d... poses) {
        field.getObject("Path").setPoses(poses);
    }

    public void updateSuperstructureState(SuperstructureState goal, SuperstructurePose currentPose) {
        elevatorheight = currentPose.elevatorHeight();

        // Limit speed based current height or goal height, whichever is higher=
        maxSpeed = goal.getData().getPose().elevatorHeight().gte(currentPose.elevatorHeight())
                // ? goal.getData().getMaxRobotSpeed()
                ? getMaxRobotSpeed(goal.getData().getPose().elevatorHeight())
                : getMaxRobotSpeed(currentPose.elevatorHeight());
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public static LinearVelocity getMaxRobotSpeed(Distance elevatorHeight) {
        return elevatorHeight.lt(speedLimitMinHeight)
                ? DriveConstants.maxSpeed
                : MetersPerSecond.of(
                        (1 - (elevatorHeight.in(Meters) - 0.5) / (ElevatorConstants.maximumHeight.in(Meters) - 0.5))
                                        * (DriveConstants.maxSpeed.in(MetersPerSecond)
                                                - DriveConstants.maxSpeedExtended.in(MetersPerSecond))
                                + DriveConstants.maxSpeedExtended.in(MetersPerSecond));
    }

    public boolean isSelected(CoralMode selection) {
        return coralSelection == selection;
    }

    public boolean isSelected(AlgaeMode selection) {
        return algaeSelection == selection;
    }

    public Command setCoralSelection(CoralMode selection) {
        return Commands.runOnce(() -> coralSelection = selection)
                .ignoringDisable(true)
                .withName("RobotStateSetCoralSelection");
    }

    public Command setAlgaeSelection(AlgaeMode selection) {
        return Commands.runOnce(() -> algaeSelection = selection)
                .ignoringDisable(true)
                .withName("RobotStateSetAlgaeSelection");
    }
}
