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
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveMeasurement;
import frc.robot.subsystems.superstructure.SuperstructurePose;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.GeometryUtil;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    private static final Rotation2d reefPoseAngleTolerance = Rotation2d.fromDegrees(25);
    private static final Rotation2d facingProcessorTolerance = Rotation2d.fromDegrees(25);
    private static final Rotation2d facingBargeTolerance = Rotation2d.fromDegrees(25);
    private static final Distance reefAreaDistanceTolerance = Meters.of(2.5);
    private static final Distance processorAreaDistanceTolerance = Meters.of(2);
    private static final Distance bargeAreaLengthTolerance = Meters.of(2);
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

    private @AutoLogOutput @Getter ReefMode reefSelection = ReefMode.L4_CORAL;
    private @AutoLogOutput @Getter AlgaeMode algaeSelection = AlgaeMode.L3;
    private @AutoLogOutput @Getter ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter ChassisSpeeds fieldVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter LinearVelocity robotSpeed = MetersPerSecond.zero();
    private @AutoLogOutput @Getter LinearVelocity maxSpeed = DriveConstants.maxSpeed;
    private Distance elevatorheight = Meters.zero();
    private boolean hasLongCoral;
    private boolean hasWideCoral;
    private boolean hasAlgae;

    public static enum ReefMode {
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL
    }

    public static enum AlgaeMode {
        PROCESSOR,
        L2,
        L3,
        BARGE
    }

    public RobotState() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("IsSimulation", Robot.isSimulation());
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static boolean isRed() {
        return !isBlue();
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

    public Rotation2d getHeadingToReef() {
        return FieldConstants.reefCenterPoint()
                .minus(getPose().getTranslation())
                .getAngle();
    }

    public boolean isFacingReef() {
        return GeometryUtil.isNear(getHeadingToReef(), getHeading(), Rotation2d.kCCW_90deg);
    }

    public boolean isFacingProcessor() {
        return GeometryUtil.isNear(
                getHeading(), FieldConstants.robotProcessorPose().getRotation(), facingProcessorTolerance);
    }

    public boolean isFacingBarge() {
        return GeometryUtil.isNear(
                getPose().getX() < FieldConstants.fieldCenter.getX() ? Rotation2d.kZero : Rotation2d.k180deg,
                getHeading(),
                facingBargeTolerance);
    }

    public boolean inReefArea() {
        return FieldConstants.reefCenterPoint().getDistance(getPose().getTranslation())
                < reefAreaDistanceTolerance.in(Meters);
    }

    public boolean inProcessorArea() {
        return FieldConstants.processorPose()
                        .getTranslation()
                        .getDistance(getPose().getTranslation())
                < processorAreaDistanceTolerance.in(Meters);
    }

    public boolean inBargeArea() {
        var translation = getPose().getTranslation();
        return MathUtil.isNear(
                        translation.getX(), FieldConstants.fieldCenter.getX(), bargeAreaLengthTolerance.in(Meters))
                && (isBlue()
                        ? translation.getY() > FieldConstants.fieldCenter.getY()
                        : translation.getY() < FieldConstants.fieldCenter.getY());
    }

    public boolean robotSpeedNominal(SuperstructureState state) {
        return robotSpeedNominal(state.getData().getPose());
    }

    public boolean robotSpeedNominal(SuperstructurePose pose) {
        return robotSpeedNominal(pose.elevatorHeight());
    }

    public boolean robotSpeedNominal(Distance elevatorHeight) {
        // Always allow downward movements
        if (elevatorheight.lte(this.elevatorheight)) {
            return true;
        }

        // Otherwise, speed must be below max speed for pose
        var maxSpeed = getMaxRobotSpeed(elevatorHeight);
        return robotSpeed.lt(maxSpeed) || robotSpeed.isNear(maxSpeed, nominalSpeedTolerance);
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

    public Optional<Pose2d> getClosesReefBranch() {
        var robotPose = poseEstimator.getEstimatedPosition();
        Pose2d closestPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (var branchPose : FieldConstants.reefBranchPoses()) {
            if (!GeometryUtil.isNear(robotPose.getRotation(), branchPose.getRotation(), reefPoseAngleTolerance)) {
                continue;
            }

            var distance = robotPose.getTranslation().getDistance(branchPose.getTranslation());
            if (distance < closestDistance) {
                closestPose = branchPose;
                closestDistance = distance;
            }
        }

        return Optional.ofNullable(closestPose);
    }

    public Optional<Pose2d> getClosestReefBranch() {
        return getClosestReefPose(FieldConstants.reefBranchPoses());
    }

    public Optional<Pose2d> getClosestReefAlgae() {
        return getClosestReefPose(FieldConstants.reefAlgaePoses());
    }

    private Optional<Pose2d> getClosestReefPose(Pose2d[] reefPoses) {
        var robotPose = poseEstimator.getEstimatedPosition();
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

    public void addDriveMeasurements(DriveMeasurement... measurements) {
        for (var measurement : measurements) {
            poseEstimator.updateWithTime(
                    measurement.timestampSeconds, measurement.gyroAngle, measurement.modulePositions);
        }

        if (measurements.length == 0) {
            return;
        }

        robotVelocity = measurements[measurements.length - 1].chassisSpeeds;
        fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getHeading());
        robotSpeed = MetersPerSecond.of(Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public void addVisionMeasurements(VisionMeasurement... measurements) {
        for (var measurement : measurements) {
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
                : MetersPerSecond.of((1 - elevatorHeight.in(Meters) / ElevatorConstants.maximumHeight.in(Meters))
                                * (DriveConstants.maxSpeed.in(MetersPerSecond)
                                        - DriveConstants.maxSpeedExtended.in(MetersPerSecond))
                        + DriveConstants.maxSpeedExtended.in(MetersPerSecond));
    }

    public boolean isSelected(ReefMode selection) {
        return reefSelection == selection;
    }

    public boolean isSelected(AlgaeMode selection) {
        return algaeSelection == selection;
    }

    public Command setReefSelection(ReefMode selection) {
        return Commands.runOnce(() -> reefSelection = selection)
                .ignoringDisable(true)
                .withName("OperatorSetReefSelection");
    }

    public Command setAlgaeSelection(AlgaeMode selection) {
        return Commands.runOnce(() -> algaeSelection = selection)
                .ignoringDisable(true)
                .withName("OperatorSetAlgaeSelection");
    }
}
