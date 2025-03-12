package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public final class ClimbConstants {
    public static final int numMotors = 1;
    public static final double reduction = 45.0;
    public static final Distance spoolRadius = Inches.of(0.39);
    public static final Distance armLength = Inches.of(10);
    public static final Voltage retractVoltage = Volts.of(10);
}
