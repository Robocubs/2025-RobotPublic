package frc.robot.subsystems.superstructure.funnel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class FunnelConstants {
    public static final double reduction = 20.0;
    public static final Distance wheelRadius = Inches.of(3.0);

    public static final AngularVelocity feedVelocity = RadiansPerSecond.of(30);
}
