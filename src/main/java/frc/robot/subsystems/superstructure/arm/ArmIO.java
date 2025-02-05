package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public Angle angle = Radians.zero();
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Current current = Amps.zero();
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setAngle(Angle angle) {}

    public default void setAngle(Angle angle, Torque feedforward) {}

    public default void setVelocity(AngularVelocity velocity, Torque feedforward) {}

    public default void stop() {}
}
