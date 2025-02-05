package frc.robot.subsystems.superstructure.arm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Torque;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

public class Arm {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Optional<Rotation2d> holdAngle = Optional.empty();

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
        // TODO: Return the measured encoder angle from the inputs
        return Radians.zero();
    }

    public boolean isNear(Angle angle) {
        // TODO: Return whether the measured encoder angle is within tolerance of the angle
        return false;
    }

    public void setAngle(Angle angle) {
        // TODO: Command the io to set the angle
    }

    public void setAngle(Angle angle, Torque feedforward) {
        // TODO: Command the io to set the angle with feedforward torque value
        // Use new angle as the target angle
    }

    public void setVelocity(AngularVelocity velocity, Torque feedforward) {
        // TODO: Command the io to set the velocity with feedforward torque value
        // Calculate the target angle based on the current angle and the velocity
    }

    public void stop() {
        // TODO: Stop the arm from moving
        // Set the goal to stop
    }

    public void hold() {
        /*
         * TODO: Set the hold angle if it is no already set.
         * Command the io to go to the hold angle.
         * Make sure that this angle does not change if there is already a hold angle.
         * All other set methods (and stop) should clear the hold angle.
         */
    }
}
