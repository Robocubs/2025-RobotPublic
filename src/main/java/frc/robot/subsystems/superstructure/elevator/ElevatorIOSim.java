package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Kilograms;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class ElevatorIOSim extends ElevatorIOHardware {
    private static final double kSimLoopPeriod = 0.005;

    private final ElevatorSim motorSim;
    private final Notifier simNotifier;
    private double lastSimTime;

    public ElevatorIOSim() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        var gearbox = DCMotor.getKrakenX60Foc(2);
        motorSim = new ElevatorSim(
                gearbox,
                reduction,
                loadMass.in(Kilograms),
                sprocketRadius.in(Meters),
                0.0,
                maximumHeight.in(Meters) * reduction,
                true,
                0.0);

        var masterSimState = masterMotor.getSimState();
        var followerSimState = followerMotor.getSimState();

        simNotifier = new Notifier(() -> {
            final var currentTime = Utils.getCurrentTimeSeconds();
            var deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            motorSim.setInputVoltage(masterSimState.getMotorVoltage());
            motorSim.update(deltaTime);

            var position = Radians.of(motorSim.getPositionMeters() / sprocketRadius.in(Meters));
            var velocity = RadiansPerSecond.of(motorSim.getVelocityMetersPerSecond() / sprocketRadius.in(Meters));

            masterSimState.setRawRotorPosition(position);
            masterSimState.setRotorVelocity(velocity);
            masterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            followerSimState.setRawRotorPosition(position);
            followerSimState.setRotorVelocity(velocity);
            followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
