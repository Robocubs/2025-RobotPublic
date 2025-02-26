package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.*;

public final class ElevatorConstants {
    public static final int numMotors = 2;
    public static final double reduction = 7.75 / 3.0;
    public static final Angle elevatorAngle = Degrees.of(85);
    public static final Distance sprocketRadius = Inches.of(0.875);
    public static final Mass loadMass = Pounds.of(23.5);
    public static final InvertedValue motorInvertedValue = InvertedValue.Clockwise_Positive;

    public static final Distance positionTolerance = Inches.of(1.0);
    public static final Distance maximumHeight = Inches.of(80.5);
    // public static final LinearVelocity maximumVelocity = FeetPerSecond.of(10);
    public static final LinearVelocity maximumVelocity = FeetPerSecond.of(10);
    // public static final LinearAcceleration maximumAcceleration = maximumVelocity.div(Seconds.of(0.1));
    public static final LinearAcceleration maximumAcceleration = maximumVelocity.div(Seconds.of(0.4));
    public static final Velocity<LinearAccelerationUnit> maximumJerk = maximumAcceleration.div(Seconds.of(0.1));
}
