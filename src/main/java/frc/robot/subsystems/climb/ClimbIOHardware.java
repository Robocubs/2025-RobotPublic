package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ClimbIOHardware implements ClimbIO {
    private static final double kT = DCMotor.getKrakenX60(numMotors).withReduction(reduction).KtNMPerAmp * numMotors;
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Climb/PositionKP", 15);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Climb/PositionKD", 10);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Climb/VelocityKP", 50);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Climb/VelocityKD", 0.1);

    protected final TalonFX motor;
    private final TalonFXConfiguration motorConfig;
    private final Servo brakeServo = new Servo(0);
    protected final Servo releaseServo = new Servo(1);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;

    private final VoltageOut voltageControlRequest = new VoltageOut(0.0);
    private final TorqueCurrentFOC torqueCurrentControlRequest = new TorqueCurrentFOC(0.0);
    private final PositionTorqueCurrentFOC positionControlRequest = new PositionTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC velocityControlRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(1);

    public ClimbIOHardware() {
        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction))
                .withSlot0(new Slot0Configs().withKP(positionKP.get()).withKD(positionKD.get()))
                .withSlot1(new Slot1Configs().withKP(velocityKP.get()).withKD(velocityKD.get()))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Amps.of(60))
                        .withPeakReverseTorqueCurrent(Amps.of(-5)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLowerLimit(Amps.of(40)));

        motor = new TalonFX(35, Constants.canivoreBusName);
        tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        torqueCurrentSignal = motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                torqueCurrentSignal);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                positionSignal, velocitySignal, voltageSignal, supplyCurrentSignal, torqueCurrentSignal);
        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.torqueCurrent = torqueCurrentSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.Slot0.kP = positionKP.get();
                    motorConfig.Slot0.kD = positionKD.get();
                    motorConfig.Slot1.kP = velocityKP.get();
                    motorConfig.Slot1.kD = velocityKD.get();
                    tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));
                },
                positionKP,
                positionKD,
                velocityKP,
                velocityKD);
    }

    @Override
    public void setBrakeServoAngle(Angle angle) {
        brakeServo.setAngle(angle.in(Degrees));
    }

    @Override
    public void setReleaseServoSpeed(double speed) {
        releaseServo.set(speed / 2 + 0.5);
    }

    @Override
    public void stopReleaseServo() {
        releaseServo.set(0.5);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageControlRequest.withOutput(voltage));
    }

    @Override
    public void setTorqueCurrent(Current current) {
        motor.setControl(torqueCurrentControlRequest.withOutput(current));
    }

    @Override
    public void setPosition(Angle position, Torque feedforward) {
        motor.setControl(
                positionControlRequest.withPosition(position).withFeedForward(feedforward.in(NewtonMeters) / kT));
    }

    @Override
    public void setVelocity(AngularVelocity velocity, Torque feedforward) {
        motor.setControl(
                velocityControlRequest.withVelocity(velocity).withFeedForward(feedforward.in(NewtonMeters) / kT));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        motor.setNeutralMode(neutralMode);
    }
}
