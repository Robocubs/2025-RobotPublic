package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public final class ElevatorConstants {
    public static final int numMotors = 2;
    public static final double reduction = 7.75 / 3.0;
    public static final Angle elevatorAngle = Degrees.of(85);
    public static final Distance sprocketRadius = Inches.of(0.875);
    public static final Mass loadMass = Pounds.of(17);
    public static final InvertedValue motorInvertedValue = InvertedValue.Clockwise_Positive;

    public static final Distance positionTolerance = Inches.of(2.0);
    public static final Distance maximumHeight = Inches.of(80.5);
}
