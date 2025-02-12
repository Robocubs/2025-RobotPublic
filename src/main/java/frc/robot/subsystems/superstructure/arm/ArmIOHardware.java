package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ArmIOHardware implements ArmIO {
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/KG", 0.4);
    private static final LoggedTunableNumber motionMagicMaxVelocity =
            new LoggedTunableNumber("Arm/MotionMagicMaxVelocity", maximumVelocity.in(RotationsPerSecond));
    private static final LoggedTunableNumber motionMagicMaxAcceleration = new LoggedTunableNumber(
            "Arm/MotionMagicMaxAcceleration", maximumAcceleration.in(RotationsPerSecondPerSecond));
    private static final LoggedTunableNumber motionMagicMaxJerk =
            new LoggedTunableNumber("Arm/MotionMagicMaxJerk", maximumJerk.in(RotationsPerSecondPerSecond.per(Second)));
    private static final LoggedTunableNumber motionMagicKV = new LoggedTunableNumber("Arm/MotionMagicKV", 8.03);
    private static final LoggedTunableNumber motionMagicKA = new LoggedTunableNumber("Arm/MotionMagicKA", 0.09);
    private static final LoggedTunableNumber motionMagicKP = new LoggedTunableNumber("Arm/MotionMagicKP", 0.0);
    private static final LoggedTunableNumber motionMagicKD = new LoggedTunableNumber("Arm/MotionMagicKD", 0.0);
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Arm/PositionKP", 0.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Arm/PositionKD", 0.0);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Arm/VelocityKP", 0.0);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Arm/VelocityKD", 0.0);
    private static final LoggedTunableBoolean useMotionMagic = new LoggedTunableBoolean("Arm/UseMotionMagic", true);

    protected final CANcoder cancoder;
    protected final TalonFX motor;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> angleSignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withSlot(0);
    private final PositionVoltage positionControlRequest = new PositionVoltage(0.0).withSlot(1);
    private final VelocityVoltage velocityControlRequest = new VelocityVoltage(0.0).withSlot(2);

    public ArmIOHardware() {
        var cancoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(Radians.zero())
                        .withSensorDirection(cancoderSensorDirection));
        cancoder = new CANcoder(10, Constants.canivoreBusName);
        tryUntilOk(() -> cancoder.getConfigurator().apply(cancoderConfig));

        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(motorInvertedValue))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(1.0)
                        .withRotorToSensorRatio(reduction)
                        .withFusedCANcoder(cancoder))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(motionMagicMaxVelocity.getAsDouble())
                        .withMotionMagicAcceleration(motionMagicMaxAcceleration.getAsDouble())
                        .withMotionMagicJerk(motionMagicMaxJerk.getAsDouble())
                        .withMotionMagicExpo_kV(8.03)
                        .withMotionMagicExpo_kA(0.2))
                .withSlot0(new Slot0Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKS(0)
                        .withKV(motionMagicKV.get())
                        .withKA(motionMagicKA.get())
                        .withKP(motionMagicKP.get())
                        .withKD(motionMagicKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
                .withSlot2(new Slot2Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(velocityKP.get())
                        .withKD(velocityKD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(maximumAngle)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(minimumAngle))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Amps.of(120))
                        .withPeakReverseTorqueCurrent(Amps.of(120)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLimitEnable(true));

        motor = new TalonFX(22, Constants.canivoreBusName);
        tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));

        angleSignal = cancoder.getAbsolutePosition();
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        closedLoopReferenceSignal = motor.getClosedLoopReference();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                angleSignal,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                closedLoopReferenceSignal);
        cancoder.optimizeBusUtilization();
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                angleSignal,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                closedLoopReferenceSignal);
        inputs.angle = angleSignal.getValue();
        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.statorCurrent = statorCurrentSignal.getValue();
        inputs.closedLoopReference = Units.rotationsToRadians(closedLoopReferenceSignal.getValue());

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicMaxVelocity.get();
                    motorConfig.MotionMagic.MotionMagicAcceleration = motionMagicMaxAcceleration.get();
                    motorConfig.MotionMagic.MotionMagicJerk = motionMagicMaxJerk.get();
                    motorConfig.MotionMagic.MotionMagicExpo_kV = motionMagicKV.get();
                    motorConfig.MotionMagic.MotionMagicExpo_kA = motionMagicKA.get();
                    motorConfig.Slot0.kG = kG.get();
                    motorConfig.Slot0.kV = motionMagicKV.get();
                    motorConfig.Slot0.kA = motionMagicKA.get();
                    motorConfig.Slot0.kP = motionMagicKP.get();
                    motorConfig.Slot0.kD = motionMagicKD.get();
                    motorConfig.Slot1.kG = kG.get();
                    motorConfig.Slot1.kP = positionKP.get();
                    motorConfig.Slot1.kD = positionKD.get();
                    motorConfig.Slot2.kG = kG.get();
                    motorConfig.Slot2.kP = velocityKP.get();
                    motorConfig.Slot2.kD = velocityKD.get();
                    tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));
                },
                kG,
                motionMagicMaxVelocity,
                motionMagicMaxAcceleration,
                motionMagicMaxJerk,
                motionMagicKV,
                motionMagicKA,
                motionMagicKP,
                motionMagicKD,
                positionKP,
                positionKD,
                velocityKP,
                velocityKD);
    }

    @Override
    public void setAngle(Angle angle) {
        if (useMotionMagic.get()) {
            motor.setControl(motionMagic.withPosition(angle));
        } else {
            setAngle(angle, Volts.zero());
        }
    }

    @Override
    public void setAngle(Angle angle, Voltage feedforward) {
        motor.setControl(positionControlRequest.withPosition(angle));
    }

    @Override
    public void setVelocity(AngularVelocity velocity, Voltage feedforward) {
        motor.setControl(velocityControlRequest.withVelocity(velocity));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
