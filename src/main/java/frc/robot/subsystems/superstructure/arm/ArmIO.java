package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public Rotation2d angle;
        public Rotation2d position;
        public double velocity;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setAngle(Rotation2d angle) {}

    public default void stop() {}
}
