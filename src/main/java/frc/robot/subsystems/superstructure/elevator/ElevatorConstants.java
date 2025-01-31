package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.units.*;
import edu.wpi.first.units.collections.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public final class ElevatorConstants {
    public static final double reduction = 2.0;
    public static final Angle elevatorAngle = Degrees.of(85);
    public static final Distance sprocketRadius = Inches.of(0.875 / 2);
    public static final Mass loadMass = Pounds.of(20);

    public static final Distance positionTolerance = Inches.of(0.5);
    public static final Distance maximumHeight = Inches.of(85);
    public static final LinearVelocity maximumVelocity = FeetPerSecond.of(10);
    public static final LinearAcceleration maximumAcceleration = maximumVelocity.div(Seconds.of(0.1));
    public static final Velocity<LinearAccelerationUnit> maximumJerk = maximumAcceleration.div(Seconds.of(0.1));
}
