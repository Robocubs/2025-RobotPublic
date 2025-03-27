package frc.robot.subsystems.superstructure.funnel;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.superstructure.rollers.Rollers;
import frc.robot.subsystems.superstructure.rollers.RollersConstants;
import frc.robot.util.tuning.LoggedTunableMeasure;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.superstructure.funnel.FunnelConstants.*;

public class Funnel {
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> feedVelocity =
            new LoggedTunableMeasure<>(
                    "Funnel/FeedVelocity",
                    Rollers.coralFeedVelocity.get().times(wheelRadius.div(RollersConstants.coralRollerRadius)));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> fastFeedVelocity =
            new LoggedTunableMeasure<>(
                    "Funnel/FastFeedVelocity",
                    Rollers.coralFastFeedVelocity.get().times(wheelRadius.div(RollersConstants.coralRollerRadius)));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> reverseFeedVelocity =
            new LoggedTunableMeasure<>(
                    "Funnel/ReverseFeedVelocity", feedVelocity.get().times(-1.0));

    private final FunnelIO io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

    private @AutoLogOutput AngularVelocity targetSpeed = RadiansPerSecond.zero();

    public Funnel(FunnelIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Funnel", inputs);
    }

    public void feed() {
        io.setVelocity(feedVelocity.get());
        targetSpeed = feedVelocity.get();
    }

    public void fastFeed() {
        io.setVelocity(fastFeedVelocity.get());
        targetSpeed = fastFeedVelocity.get();
    }

    public void reverseFeed() {
        io.setVelocity(reverseFeedVelocity.get());
        targetSpeed = reverseFeedVelocity.get();
    }

    public void stop() {
        io.stop();
        targetSpeed = RadiansPerSecond.zero();
    }
}
