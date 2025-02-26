package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.CustomDCMotor;
import frc.robot.util.simulation.SimNotifier;
import org.littletonrobotics.junction.Logger;

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

        var motorSign = motorInvertedValue == InvertedValue.CounterClockwise_Positive ? 1 : -1;
        var cancoderSign = cancoderSensorDirection == SensorDirectionValue.CounterClockwise_Positive ? 1 : -1;

        if (cancoder != null) {
            var cancoderSimState = cancoder.getSimState();
            cancoderSimState.setRawPosition(
                    initialAngle.minus(Radians.of(encoderOffset.get())).times(cancoderSign));
        }

        var motorSimState = motor.getSimState();
        motorSimState.setRawRotorPosition(initialAngle);

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage() * motorSign);
            motorSim.update(deltaTime.in(Seconds));

            var angle = Radians.of(motorSim.getAngleRads());
            var velocity = RadiansPerSecond.of(motorSim.getVelocityRadPerSec());

            Logger.recordOutput("Arm/SimArmAngle", angle);
            Logger.recordOutput("Arm/SimArmVelocity", velocity);

            if (cancoder != null) {
                var cancoderSimState = cancoder.getSimState();
                cancoderSimState.setRawPosition(
                        angle.minus(Radians.of(encoderOffset.get())).times(cancoderSign));
                cancoderSimState.setVelocity(velocity.times(cancoderSign));
                cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            }

            motorSimState.setRawRotorPosition(angle.times(reduction));
            motorSimState.setRotorVelocity(velocity.times(reduction));
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
