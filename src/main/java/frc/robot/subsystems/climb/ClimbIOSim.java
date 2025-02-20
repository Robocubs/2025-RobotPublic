package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climb.ClimbConstants.*;

public class ClimbIOSim extends ClimbIOHardware {
    private Angle position;

    public ClimbIOSim() {
        // TODO: Implement custom simulation code to handle the constant force spring
        // Maybe this can be done with arm simulation?

        var gearbox = DCMotor.getKrakenX60Foc(numMotors);
        var motorSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        gearbox, Pounds.of(150).in(Kilograms) * armLength.in(Meters) * armLength.in(Meters), reduction),
                gearbox);

        var motorSimState = motor.getSimState();
        position = Radians.zero();

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var velocity = motorSim.getAngularVelocity().times(reduction);
            position = position.plus(velocity.times(deltaTime));

            motorSimState.setRawRotorPosition(position);
            motorSimState.setRotorVelocity(velocity);
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
