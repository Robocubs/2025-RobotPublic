package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class ElevatorIOSim extends ElevatorIOHardware {
    private final ElevatorSim motorSim;

    public ElevatorIOSim() {
        var gearbox = DCMotor.getKrakenX60Foc(numMotors);
        motorSim = new ElevatorSim(
                gearbox,
                reduction,
                loadMass.in(Kilograms),
                sprocketRadius.in(Meters),
                0.0,
                maximumHeight.in(Meters),
                true,
                0.0);

        var motorSign = motorInvertedValue == InvertedValue.CounterClockwise_Positive ? 1 : -1;
        var masterSimState = masterMotor.getSimState();
        var followerSimState = followerMotor.getSimState();

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(masterSimState.getMotorVoltage() * motorSign);
            motorSim.update(deltaTime.in(Seconds));

            var position = Radians.of(motorSim.getPositionMeters() / sprocketRadius.in(Meters) * reduction * motorSign);
            var velocity = RadiansPerSecond.of(
                    motorSim.getVelocityMetersPerSecond() / sprocketRadius.in(Meters) * reduction * motorSign);

            masterSimState.setRawRotorPosition(position);
            masterSimState.setRotorVelocity(velocity);
            masterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            followerSimState.setRawRotorPosition(position);
            followerSimState.setRotorVelocity(velocity);
            followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }

    @Override
    public void zeroPosition(Distance position) {
        motorSim.setState(position.in(Meters), 0);
        masterMotor.getSimState().setRawRotorPosition(0);
        followerMotor.getSimState().setRawRotorPosition(0);
        super.zeroPosition(position);
    }
}
