package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public Distance masterPosition = Meters.zero();
        public LinearVelocity masterVelocity = MetersPerSecond.zero();
        public Voltage masterVoltage = Volts.zero();
        public Current masterSupplyCurrent = Amps.zero();
        public Current masterTorqueCurrent = Amps.zero();

        public Distance followerPosition = Meters.zero();
        public LinearVelocity followerVelocity = MetersPerSecond.zero();
        public Voltage followerVoltage = Volts.zero();
        public Current followerSupplyCurrent = Amps.zero();
        public Current followerTorqueCurrent = Amps.zero();

        public double closedLoopReference = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(Voltage voltage) {}

    public default void setTorqueCurrent(Current current) {}

    public default void setPosition(Distance position) {}

    public default void setPosition(Distance position, Force feedforward) {}

    public default void setVelocity(LinearVelocity velocity, Force feedforward) {}

    public default void zeroPosition() {}

    public default void stop() {}
}
