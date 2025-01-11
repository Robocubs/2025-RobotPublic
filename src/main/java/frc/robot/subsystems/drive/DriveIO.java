package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public class DriveIOInputs {
        public SwerveDriveInputs[] inputs = new SwerveDriveInputs[0];
        public Rotation3d rotation = Rotation3d.kZero;
    }

    public default void updateInputs(DriveIOInputs inputs) {}

    public default void setControl(SwerveRequest request) {}

    public default void resetRotation(Rotation2d rotation2d) {}
}
