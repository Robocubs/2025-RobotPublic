package frc.robot.subsystems.climb;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.climb.ClimbConstants.*;

public class Climb extends SubsystemBase {
    private static final LoggedTunableNumber climbedPosition = new LoggedTunableNumber("Climb/ClimbedPosition", 60);
    private static final LoggedTunableNumber brakeEngagedAngle =
            new LoggedTunableNumber("Climb/BrakeEngagedAngle", 120.0);
    private static final LoggedTunableNumber brakeDisengagedAngle =
            new LoggedTunableNumber("Climb/BrakeDisengagedAngle", 60.0);
    private static final LoggedTunableNumber minBrakeSpeed = new LoggedTunableNumber("Climb/ClimbedPosition", -0.2);
    private static final LoggedTunableNumber zeroAmpsLimit = new LoggedTunableNumber("Climb/ZeroAmpsLimit", 0.0);

    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private Optional<Angle> zeroedPosition = Optional.empty();

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public Command deploy() {
        return sequence(
                        /*
                         * TODO:
                         * - Run the zero routine if it hasn't been run yet
                         * - Move the motor to the zero position
                         * - Engage the brake
                         */
                        )
                .withName("ClimbDeploy");
    }

    public Command retract() {
        return sequence(
                        /*
                         * TODO:
                         * - Run the zero routine if it hasn't been run yet
                         * - Engage the brake
                         * - Set the motor to full volts until the climb position is reached
                         */
                        )
                .withName("ClimbClimb");
    }

    public Command zero() {
        return sequence(
                        /*
                         * TODO:
                         * - Set the motor to forward at -2.0 volts until the current is above 10 amps
                         * - Set the motor to reverse at 2.0 volts until the current is below 10 amps
                         * - Stop the motor and set the zeroed position to the current position
                         */
                        )
                .withName("ClimbZero");
    }

    public Command stop() {
        return run(() -> {
                    /*
                     * TODO:
                     * - Command the io to stop
                     * - If the speed is greater than the minimum brake speed, engage the brake
                     */
                })
                .withName("ClimbStop");
    }
}
