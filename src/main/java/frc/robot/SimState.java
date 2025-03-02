package frc.robot;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;

public class SimState {
    private final LoggedTunableMeasure<DistanceUnit, Distance> coralSensorDistance =
            new LoggedTunableMeasure<>("SimState/CoralSensorDistance", Meters.of(0.6));
    private final LoggedTunableMeasure<DistanceUnit, Distance> algaeSensorDistance =
            new LoggedTunableMeasure<>("SimState/AlgaeSensorDistance", Meters.of(0.6));
    private final LoggedTunableMeasure<DistanceUnit, Distance> elevatorSensorDistance =
            new LoggedTunableMeasure<>("SimState/ElevatorSensorDistance", Meters.of(0.6));

    public Distance getCoralSensorDistance() {
        return coralSensorDistance.get();
    }

    public Distance getAlgaeSensorDistance() {
        return algaeSensorDistance.get();
    }

    public Distance getElevatorSensorDistance() {
        return elevatorSensorDistance.get();
    }
}
