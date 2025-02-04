package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.*;

public class ArmConstants {
    public static final double reduction = 100.0;
    public static final Mass mass = Pounds.of(6.44);
    public static final Distance length = Inches.of(17.323);
    public static final Distance cg = Inches.of(10.941);
    public static final MomentOfInertia moi =
            Pounds.mult(InchesPerSecond).mult(Inches).mult(RadiansPerSecond).of(1100.53);
    public static final Angle angleTolerance = Degrees.of(1);
    public static final Angle maximumAngle = Degrees.of(270);
    public static final Angle minimumAngle = Degrees.of(-90);
    public static final AngularVelocity maximumVelocity = RotationsPerSecond.of(10);
    public static final AngularAcceleration maximumAcceleration = maximumVelocity.div(Seconds.of(0.5));
    public static final Velocity<AngularAccelerationUnit> maximumJerk = maximumAcceleration.div(Seconds.of(0.1));
}
