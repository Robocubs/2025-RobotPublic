package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

public class ArmIOSim extends ArmIOHardware {
    private static final double kSimLoopPeriod = 0.005;

    private final SingleJointedArmSim motorSim;
    private final Notifier simNotifier;
    private double lastSimTime;

    public ArmIOSim() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        // var gearbox = CustomDCMotor.getKrakenX44(1);
        var gearbox = DCMotor.getKrakenX60Foc(1);
        motorSim = new SingleJointedArmSim(
                gearbox,
                reduction,
                moi.in(KilogramSquareMeters),
                length.in(Meters),
                minimumAngle.in(Radians),
                maximumAngle.in(Radians),
                true,
                Degrees.of(90).in(Radians));
        motorSim.setState(Degrees.of(95).in(Radians), 0);

        var cancoderSimState = cancoder.getSimState();
        cancoderSimState.setRawPosition(Degrees.of(90));

        var motorSimState = motor.getSimState();
        motorSimState.setRawRotorPosition(Degrees.of(90).times(reduction));

        simNotifier = new Notifier(() -> {
            var currentTime = Utils.getCurrentTimeSeconds();
            var deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime);

            var angle = Radians.of(motorSim.getAngleRads() * reduction);
            var velocity = RadiansPerSecond.of(motorSim.getVelocityRadPerSec() * reduction);

            cancoderSimState.setRawPosition(angle.div(reduction));
            cancoderSimState.setVelocity(velocity.div(reduction));
            cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            motorSimState.setRawRotorPosition(angle);
            motorSimState.setRotorVelocity(velocity);
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
