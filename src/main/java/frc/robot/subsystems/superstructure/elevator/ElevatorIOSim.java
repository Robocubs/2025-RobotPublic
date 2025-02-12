package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class ElevatorIOSim extends ElevatorIOHardware {
    public ElevatorIOSim() {
        var gearbox = DCMotor.getKrakenX60Foc(numMotors);
        var motorSim = new ElevatorSim(
                gearbox,
                reduction,
                loadMass.in(Kilograms),
                sprocketRadius.in(Meters),
                0.0,
                maximumHeight.in(Meters),
                true,
                0.0);

        var masterSimState = masterMotor.getSimState();
        var followerSimState = followerMotor.getSimState();

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(masterSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var position = Radians.of(motorSim.getPositionMeters() / sprocketRadius.in(Meters) * reduction);
            var velocity =
                    RadiansPerSecond.of(motorSim.getVelocityMetersPerSecond() / sprocketRadius.in(Meters) * reduction);

            masterSimState.setRawRotorPosition(position);
            masterSimState.setRotorVelocity(velocity);
            masterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            followerSimState.setRawRotorPosition(position);
            followerSimState.setRotorVelocity(velocity);
            followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
