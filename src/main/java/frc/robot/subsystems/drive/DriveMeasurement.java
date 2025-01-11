package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DriveMeasurement {
    public final double timestampSeconds;
    public final Rotation2d gyroAngle;
    public final SwerveModulePosition[] modulePositions;
    public final ChassisSpeeds chassisSpeeds;

    public DriveMeasurement(
            double timestamp,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            ChassisSpeeds chassisSpeeds) {
        this.timestampSeconds = timestamp;
        this.gyroAngle = gyroAngle;
        this.modulePositions = modulePositions;
        this.chassisSpeeds = chassisSpeeds;
    }
}
