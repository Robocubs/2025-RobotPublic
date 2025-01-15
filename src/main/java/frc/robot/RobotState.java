package frc.robot;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveMeasurement;
import frc.robot.subsystems.vision.VisionMeasurement;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            Rotation2d.kZero,
            Stream.generate(SwerveModulePosition::new).limit(4).toArray(SwerveModulePosition[]::new),
            Pose2d.kZero,
            DriveConstants.odometryStdDevs,
            VecBuilder.fill(1, 1, 1) // Vision std devs will be passed in with vision measurements
            );

    private ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private ChassisSpeeds fieldVelocity = new ChassisSpeeds();

    public RobotState() {}

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static boolean isRed() {
        return !isBlue();
    }

    @AutoLogOutput(key = "RobotState/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Optional<Pose2d> getPose(double timestamp) {
        return poseEstimator.sampleAt(timestamp);
    }

    @AutoLogOutput(key = "RobotState/Heading")
    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    @AutoLogOutput(key = "RobotState/RobotVelocity")
    public ChassisSpeeds getRobotVelocity() {
        return fieldVelocity;
    }

    @AutoLogOutput(key = "RobotState/FieldVelocity")
    public ChassisSpeeds getFieldVelocity() {
        return fieldVelocity;
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
    }

    public void addVisionMeasurements(VisionMeasurement... measurements) {
        for (var measurement : measurements) {
            poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestampSeconds);
        }
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }
}
