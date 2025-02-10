package frc.robot.subsystems.superstructure.rollers;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Torque;
import frc.robot.Constants;
import frc.robot.util.booleans.LatchedBoolean;
import frc.robot.util.booleans.ThresholdLatchedBoolean;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;

public class Rollers {
    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private final Supplier<Angle> armAngle;
    private final ThresholdLatchedBoolean longCoralDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            coralDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final ThresholdLatchedBoolean algaeDetected = ThresholdLatchedBoolean.fromThresholdTolerance(
            algaeDetectionDistance.in(Meters), detectionDistanceTolerance.in(Meters), false);
    private final LatchedBoolean wideCoralDetected = new LatchedBoolean(true);

    private Optional<Angle> autoFeedCoralPosition = Optional.empty();
    private Optional<Angle> autoIntakeCoralPosition = Optional.empty();
    private Optional<Angle> autoIntakeAlgaePosition = Optional.empty();

    @AutoLogOutput
    private State state = State.STOPPED;

    public static enum State {
        STOPPED,
        AUTO_FEED_CORAL,
        AUTO_INTAKE_CORAL,
        AUTO_INTAKE_ALGAE,
        CORAL_FORWARD,
        CORAL_REVERSE,
        ALGAE_FORWARD,
        HOLD
    }

    public Rollers(RollersIO io, Supplier<Angle> armAngle) {
        this.io = io;
        this.armAngle = armAngle;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);

        /*
         * TODO: Implement detection logic for each detector
         * 1. Update detection booleans with the sensor distance
         * 2. If detected is true and auto intake position is empty,
         *    set the auto intake position to the current position plus/minus the auto intake distance
         * 3. If detected is false,
         *    set the auto intake position to empty
         * 4. If the state is AUTO_INTAKE_CORAL and a coral is detected, set the wideCoralDetected to true
         */

        /*
         * TOOD: Implement state logic
         *
         * Auto states:
         * 1. If the auto position is present, Command the io to go to the position
         * 2. If the auto position is not present, Command the io to run the intake velocity
         *
         * Hold state:
         * 1. If any auto position exists, hold that position
         * 2. Otherwise, set the io to stop
         */
        switch (state) {
            case AUTO_FEED_CORAL:
                break;
            case AUTO_INTAKE_CORAL:
                break;
            case AUTO_INTAKE_ALGAE:
                break;
            case CORAL_FORWARD:
                break;
            case CORAL_REVERSE:
                break;
            case ALGAE_FORWARD:
                break;
            case HOLD:
                break;
            case STOPPED:
                break;
            default:
                break;
        }
    }

    @AutoLogOutput
    public boolean longCoralDetected() {
        // TODO: implement
        return false;
    }

    @AutoLogOutput
    public boolean algaeDetected() {
        // TODO: implement
        return false;
    }

    public void setState(State state) {
        // TODO: implement
        // If the state is not AUTO_INTAKE_CORAL or HOLD, reset the wideCoralDetected latched boolean
    }

    public void stop() {
        // TODO: implement
    }

    private Torque coralRollerFeedForward() {
        return NewtonMeters.of(9.81
                * Constants.coralMass.in(Kilograms)
                * Math.cos(armAngle.get().in(Radians) + coralAngleRelativeToArm.in(Radians) - Math.PI / 2)
                / coralRollerReduction
                / 2.0);
    }

    private Torque hybridRollerCoralFeedForward() {
        return NewtonMeters.of(9.81
                * Constants.coralMass.in(Kilograms)
                * Math.cos(armAngle.get().in(Radians) + coralAngleRelativeToArm.in(Radians) - Math.PI / 2)
                / hybridRollerReduction
                / 2.0);
    }

    private Torque hybridRollerAlgaeFeedForward() {
        return NewtonMeters.of(9.81
                * Constants.algaeMass.in(Kilograms)
                * Math.cos(armAngle.get().in(Radians) + algaeAngleRelativeToArm.in(Radians) - Math.PI / 2)
                / hybridRollerReduction
                / 2.0);
    }
}
