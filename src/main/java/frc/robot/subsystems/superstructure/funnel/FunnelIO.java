package frc.robot.subsystems.superstructure.funnel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface FunnelIO {
    @AutoLog
    public class FunnelIOInputs {
        public Angle masterPosition = Radians.zero();
        public AngularVelocity masterVelocity = RadiansPerSecond.zero();
        public Voltage masterVoltage = Volts.zero();
        public Current masterSupplyCurrent = Amps.zero();
        public Current masterStatorCurrent = Amps.zero();

        public Angle followerPosition = Radians.zero();
        public AngularVelocity followerVelocity = RadiansPerSecond.zero();
        public Voltage followerVoltage = Volts.zero();
        public Current followerSupplyCurrent = Amps.zero();
        public Current followerStatorCurrent = Amps.zero();

        public double closedLoopReference = 0.0;
    }

    public default void updateInputs(FunnelIOInputs inputs) {}

    public default void setVoltage(Voltage voltage) {}

    public default void setVelocity(AngularVelocity velocity) {}

    public default void stop() {}
}
