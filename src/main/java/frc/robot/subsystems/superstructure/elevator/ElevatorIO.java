package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public Distance position = Meters.zero();
        public LinearVelocity velocity = MetersPerSecond.zero();
        public Current masterCurrent = Amps.zero();
        public Current followerCurrent = Amps.zero();
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(Distance position) {}

    public default void setPosition(Distance position, Force feedforward) {}

    public default void setVelocity(LinearVelocity velocity, Force feedforward) {}

    public default void stop() {}
}
