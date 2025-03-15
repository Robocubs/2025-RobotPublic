package frc.robot.subsystems.superstructure.rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface RollersIO {
    @AutoLog
    public class RollersIOInputs {
        public Angle coralPosition = Radians.zero();
        public AngularVelocity coralVelocity = RadiansPerSecond.zero();
        public Voltage coralVoltage = Volts.zero();
        public Current coralSupplyCurrent = Amps.zero();
        public Current coralTorqueCurrent = Amps.zero();
        public double coralClosedLoopReferenceSignal = 0.0;

        public Angle hybridPosition = Radians.zero();
        public AngularVelocity hybridVelocity = RadiansPerSecond.zero();
        public Voltage hybridVoltage = Volts.zero();
        public Current hybridSupplyCurrent = Amps.zero();
        public Current hybridTorqueCurrent = Amps.zero();
        public double hybridClosedLoopReferenceSignal = 0.0;

        public Distance coralDetectorDistance = Meters.zero();
        public Distance algaeDetectorDistance = Meters.zero();
        public Distance elevatorDetectorDistance = Meters.zero();
        public double elevatorSignalStrength = 0;

        public Distance funnelDetectorDistance = Meters.zero();
        public double funnelSignalStrength = 0;
    }

    public default void updateInputs(RollersIOInputs inputs) {}

    public default void setCoralVelocity(AngularVelocity velocity) {}

    public default void setCoralPosition(Angle position) {}

    public default void stopCoral() {}

    public default void setHybridVelocity(AngularVelocity velocity) {}

    public default void setHybridPosition(Angle position) {}

    public default void stopHybrid() {}
}
