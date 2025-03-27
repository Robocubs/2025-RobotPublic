package frc.robot.subsystems.superstructure.rollers;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.SimState;
import frc.robot.util.CustomDCMotor;
import frc.robot.util.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;

public class RollersIOSim extends RollersIOHardware {
    private Angle motorPosition = Radians.zero();

    public RollersIOSim(SimState simState) {
        var gearbox = CustomDCMotor.getKrakenX44(1);
        var motorSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        gearbox,
                        Constants.algaeMass.in(Kilograms) * Math.pow(coralRollerRadius.in(Meters) / 2.0, 2),
                        reduction),
                gearbox);

        var motorSimState = motor.getSimState();
        var coralCanrageSimState = coralCanrange.getSimState();
        var algaeCanrangeSimState = algaeCanrange.getSimState();
        var elevatorCanrangeSimState = elevatorCanrange.getSimState();
        var funnelCanrangeSimState = funnelCanrange.getSimState();

        SimNotifier.register(deltaTime -> {
            var batteryVoltage = RobotController.getBatteryVoltage();

            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var motorVelocity = motorSim.getAngularVelocity().times(reduction);
            motorPosition = motorPosition.plus(motorVelocity.times(deltaTime));

            motorSimState.setRawRotorPosition(motorPosition);
            motorSimState.setRotorVelocity(motorVelocity);
            motorSimState.setSupplyVoltage(batteryVoltage);

            coralCanrageSimState.setDistance(simState.getCoralSensorDistance());
            coralCanrageSimState.setSupplyVoltage(batteryVoltage);
            algaeCanrangeSimState.setDistance(simState.getAlgaeSensorDistance());
            algaeCanrangeSimState.setSupplyVoltage(batteryVoltage);
            elevatorCanrangeSimState.setDistance(simState.getElevatorSensorDistance());
            elevatorCanrangeSimState.setSupplyVoltage(batteryVoltage);
            funnelCanrangeSimState.setDistance(simState.getFunnelSensorDistance());
            funnelCanrangeSimState.setSupplyVoltage(batteryVoltage);
        });
    }
}
