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
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage voltage = Volts.zero();
        public Current supplyCurrent = Amps.zero();
        public Current torqueCurrent = Amps.zero();
        public double closedLoopReferenceSignal = 0.0;

        public Distance coralDetectorDistance = Meters.zero();
        public double coralSignalStrengthSignal = 0;
        public Distance algaeDetectorDistance = Meters.zero();
        public double algaeSignalStrengthSignal = 0;
        public Distance elevatorDetectorDistance = Meters.zero();
        public double elevatorSignalStrength = 0;

        public Distance funnelDetectorDistance = Meters.zero();
        public double funnelSignalStrength = 0;
    }

    public default void updateInputs(RollersIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity) {}

    public default void setPosition(Angle position) {}

    public default void stop() {}
}
