package frc.robot.subsystems.superstructure.arm;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

public class Arm {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Optional<Angle> holdAngle = Optional.empty();

    @AutoLogOutput
    private Angle targetAngle = Radians.zero();

    public Arm(ArmIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public Angle getAngle() {
        return inputs.angle;
    }

    public boolean isNear(Angle angle) {
        return this.inputs.angle.isNear(angle, angleTolerance);
    }

    @AutoLogOutput
    public boolean atTarget() {
        return isNear(targetAngle);
    }

    public void setAngle(Angle angle) {
        holdAngle = Optional.empty();
        targetAngle = angle;
        io.setAngle(angle);
    }

    public void setAngle(Angle angle, Voltage feedforward) {
        holdAngle = Optional.empty();
        targetAngle = angle;
        io.setAngle(angle, feedforward);
    }

    public void setVelocity(AngularVelocity velocity, Voltage feedforward) {
        holdAngle = Optional.empty();
        targetAngle = inputs.angle.plus(velocity.times(Constants.mainLoopPeriod));
        io.setVelocity(velocity, feedforward);
    }

    public void stop() {
        holdAngle = Optional.empty();
        io.stop();
    }

    public void hold() {
        if (holdAngle.isEmpty()) {
            holdAngle = Optional.of(inputs.angle);
        }

        targetAngle = holdAngle.get();
        io.setAngle(holdAngle.get());
    }
}
