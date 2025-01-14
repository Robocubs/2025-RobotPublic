package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public double position;
        public double velocity;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void stop() {}
}
