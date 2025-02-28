package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public Angle angle = Radians.zero();
        public Angle rotorPosition = Radians.zero();
        public AngularVelocity rotorVelocity = RadiansPerSecond.zero();
        public Voltage voltage = Volts.zero();
        public Current supplyCurrent = Amps.zero();
        public Current statorCurrent = Amps.zero();
        public double closedLoopReference = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(Voltage voltage) {}

    public default void setAngle(Angle angle) {}

    public default void setAngle(Angle angle, Voltage feedforward) {}

    public default void setVelocity(AngularVelocity velocity, Voltage feedforward) {}

    public default void stop() {}

    public default void setNeutralMode(NeutralModeValue neutralMode) {}
}
