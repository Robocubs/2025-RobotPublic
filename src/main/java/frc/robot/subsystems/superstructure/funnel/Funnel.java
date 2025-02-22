package frc.robot.subsystems.superstructure.funnel;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.superstructure.funnel.FunnelConstants.*;

public class Funnel {
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
        io.setVelocity(feedVelocity);
        targetSpeed = feedVelocity;
    }

    public void fastFeed() {
        io.setVelocity(fastFeedVelocity);
        targetSpeed = fastFeedVelocity;
    }

    public void reverseFeed() {
        io.setVelocity(reverseFeedVelocity);
        targetSpeed = reverseFeedVelocity;
    }

    public void stop() {
        io.stop();
        targetSpeed = RadiansPerSecond.zero();
    }
}
