package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.CustomDCMotor;
import frc.robot.util.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

public class ArmIOSim extends ArmIOHardware {
    public ArmIOSim() {
        var initialAngle = Degrees.of(90);
        var gearbox = CustomDCMotor.getKrakenX44(numMotors);
        var motorSim = new SingleJointedArmSim(
                gearbox,
                reduction,
                moi.in(KilogramSquareMeters),
                length.in(Meters),
                minimumAngle.in(Radians),
                maximumAngle.in(Radians),
                true,
                initialAngle.in(Radians),
                0.001,
                0.0);

        var cancoderSimState = cancoder.getSimState();
        cancoderSimState.setRawPosition(initialAngle.div(reduction));

        var motorSimState = motor.getSimState();
        motorSimState.setRawRotorPosition(initialAngle);

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var angle = Radians.of(motorSim.getAngleRads());
            var velocity = RadiansPerSecond.of(motorSim.getVelocityRadPerSec());

            cancoderSimState.setRawPosition(angle);
            cancoderSimState.setVelocity(velocity);
            cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            motorSimState.setRawRotorPosition(angle.times(reduction));
            motorSimState.setRotorVelocity(velocity.times(reduction));
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
