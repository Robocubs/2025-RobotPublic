package frc.robot.subsystems.superstructure.funnel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.rollers.RollersConstants;

import static edu.wpi.first.units.Units.*;

public class FunnelConstants {
    public static final double reduction = 20.0;
    public static final Distance wheelRadius = Inches.of(3.0);

    public static final AngularVelocity feedVelocity = RadiansPerSecond.of(RollersConstants.coralFeedDistance.in(Meters)
            / RollersConstants.coralFeedTime.in(Seconds)
            / wheelRadius.in(Meters));
    public static final AngularVelocity fastFeedVelocity = RotationsPerSecond.of(100
            / RollersConstants.coralRollerReduction
            * RollersConstants.coralRollerRadius.in(Meters)
            / wheelRadius.in(Meters));
    public static final AngularVelocity reverseFeedVelocity =
            RadiansPerSecond.of(RollersConstants.coralFeedDistance.in(Meters)
                    / RollersConstants.coralFeedTime.in(Seconds)
                    / wheelRadius.in(Meters));
}
