package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDriveInputs implements StructSerializable {
    public final Pose2d pose;
    public final ChassisSpeeds speeds;
    public final SwerveModuleState[] moduleStates;
    public final SwerveModuleState[] moduleTargets;
    public final SwerveModulePosition[] modulePositions;
    public final Rotation2d rawHeading;
    public final double timestamp;
    public final double odometryPeriod;
    public final int successfulDaqs;
    public final int failedDaqs;

    public SwerveDriveInputs(
            Pose2d pose,
            ChassisSpeeds speeds,
            SwerveModuleState[] moduleStates,
            SwerveModuleState[] moduleTargets,
            SwerveModulePosition[] modulePositions,
            Rotation2d rawHeading,
            double timestamp,
            double odometryPeriod,
            int successfulDaqs,
            int failedDaqs) {
        this.pose = pose;
        this.speeds = speeds;
        this.moduleStates = moduleStates;
        this.moduleTargets = moduleTargets;
        this.modulePositions = modulePositions;
        this.rawHeading = rawHeading;
        this.timestamp = timestamp;
        this.odometryPeriod = odometryPeriod;
        this.successfulDaqs = successfulDaqs;
        this.failedDaqs = failedDaqs;
    }

    public static SwerveDriveInputs fromSwerveDriveState(SwerveDriveState state) {
        return new SwerveDriveInputs(
                state.Pose,
                state.Speeds,
                state.ModuleStates,
                state.ModuleTargets,
                state.ModulePositions,
                state.RawHeading,
                toFpgaTimestamp(state.Timestamp),
                state.OdometryPeriod,
                state.SuccessfulDaqs,
                state.FailedDaqs);
    }

    private static double toFpgaTimestamp(double timestamp) {
        return timestamp + Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
    }

    /** SwerveDriveInputs struct for serialization. */
    public static final SwerveDriveInputsStruct struct = new SwerveDriveInputsStruct();
}
