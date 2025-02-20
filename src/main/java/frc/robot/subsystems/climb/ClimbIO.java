package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage votage = Volts.zero();
        public Current supplyCurrent = Amps.zero();
        public Current torqueCurrent = Amps.zero();
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setBrakeServoAngle(Angle angle) {}

    public default void setVoltage(Voltage voltage) {}

    public default void setTorqueCurrent(Current current) {}

    public default void setPosition(Angle position, Torque feedforward) {}

    public default void zeroPosition() {}

    public default void stop() {}
}
