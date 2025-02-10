package frc.robot.subsystems.superstructure.rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface RollersIO {
    @AutoLog
    public class RollersIOInputs {
        public Angle coralPosition = Radians.zero();
        public AngularVelocity coralVelocity = RadiansPerSecond.zero();
        public Voltage coralVoltage = Volts.zero();
        public Current coralInputCurrent = Amps.zero();
        public Current coralTorqueCurrent = Amps.zero();

        public Angle hybridPosition = Radians.zero();
        public AngularVelocity hybridVelocity = RadiansPerSecond.zero();
        public Voltage hybridVoltage = Volts.zero();
        public Current hybridInputCurrent = Amps.zero();
        public Current hybridTorqueCurrent = Amps.zero();

        public Distance coralDetectorDistance = Meters.zero();
        public Distance algaeDetectorDistance = Meters.zero();
    }

    public default void updateInputs(RollersIOInputs inputs) {}

    public default void setCoralVelocity(AngularVelocity coralVelocity, AngularVelocity hybridVelocity) {}

    public default void setCoralPosition(Angle position, Torque feedforward) {}

    public default void stopCoral() {}

    public default void setHybridVelocity(AngularVelocity coralVelocity, AngularVelocity hybridVelocity) {}

    public default void setHybridPosition(Angle position, Torque feedforward) {}

    public default void stopHybrid() {}
}
