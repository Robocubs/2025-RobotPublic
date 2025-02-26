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
    private Angle coralMotorPosition = Radians.zero();
    private Angle hybridMotorPosition = Radians.zero();

    public RollersIOSim(SimState simState) {
        var coralGearbox = CustomDCMotor.getKrakenX44(1);
        var coralMotorSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        coralGearbox,
                        Constants.algaeMass.in(Kilograms) * Math.pow(coralRollerRadius.in(Meters) / 2.0, 2),
                        coralRollerReduction),
                coralGearbox);

        var hybridGearbox = CustomDCMotor.getKrakenX44(1);
        var hybridMotorSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        hybridGearbox,
                        Constants.algaeMass.in(Kilograms) * Math.pow(hybridRollerRadius.in(Meters) / 2.0, 2),
                        hybridRollerReduction),
                hybridGearbox);

        var coralMotorSimState = coralMotor.getSimState();
        var hybridMotorSimState = hybridMotor.getSimState();
        var coralCanrageSimState = coralCanrange.getSimState();
        var algaeCanrangeSimState = algaeCanrange.getSimState();

        SimNotifier.register(deltaTime -> {
            var batteryVoltage = RobotController.getBatteryVoltage();

            coralMotorSim.setInputVoltage(coralMotorSimState.getMotorVoltage());
            coralMotorSim.update(deltaTime.in(Seconds));

            var coralMotorVelocity = coralMotorSim.getAngularVelocity().times(coralRollerReduction);
            coralMotorPosition = coralMotorPosition.plus(coralMotorVelocity.times(deltaTime));

            coralMotorSimState.setRawRotorPosition(coralMotorPosition);
            coralMotorSimState.setRotorVelocity(coralMotorVelocity);
            coralMotorSimState.setSupplyVoltage(batteryVoltage);

            hybridMotorSim.setInputVoltage(hybridMotorSimState.getMotorVoltage());
            hybridMotorSim.update(deltaTime.in(Seconds));

            var hybridMotorVelocity = hybridMotorSim.getAngularVelocity().times(hybridRollerReduction);
            hybridMotorPosition = hybridMotorPosition.plus(hybridMotorVelocity.times(deltaTime));

            hybridMotorSimState.setRawRotorPosition(hybridMotorPosition);
            hybridMotorSimState.setRotorVelocity(hybridMotorVelocity);
            hybridMotorSimState.setSupplyVoltage(batteryVoltage);

            coralCanrageSimState.setDistance(simState.getCoralSensorDistance());
            coralCanrageSimState.setSupplyVoltage(batteryVoltage);
            algaeCanrangeSimState.setDistance(simState.getAlgaeSensorDistance());
            algaeCanrangeSimState.setSupplyVoltage(batteryVoltage);
        });
    }
}
