package frc.robot.subsystems.superstructure.arm;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ArmIOHardware implements ArmIO {
    protected static final FeedbackSensorSourceValue feedbackSensorSource = Constants.robot == RobotType.SIM_BOT
            ? FeedbackSensorSourceValue.FusedCANcoder
            : FeedbackSensorSourceValue.FusedCANdiPWM1;
    private static final LoggedTunableNumber encoderOffset = new LoggedTunableNumber("Arm/EncoderOffset", 0.25);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/KG", 0.4);
    private static final LoggedTunableNumber motionMagicMaxVelocity =
            new LoggedTunableNumber("Arm/MotionMagicMaxVelocity", maximumVelocity.in(RotationsPerSecond));
    private static final LoggedTunableNumber motionMagicMaxAcceleration = new LoggedTunableNumber(
            "Arm/MotionMagicMaxAcceleration", maximumAcceleration.in(RotationsPerSecondPerSecond));
    private static final LoggedTunableNumber motionMagicMaxJerk =
            new LoggedTunableNumber("Arm/MotionMagicMaxJerk", maximumJerk.in(RotationsPerSecondPerSecond.per(Second)));
    private static final LoggedTunableNumber motionMagicKV = new LoggedTunableNumber("Arm/MotionMagicKV", 8.03);
    private static final LoggedTunableNumber motionMagicKA = new LoggedTunableNumber("Arm/MotionMagicKA", 0.09);
    private static final LoggedTunableNumber motionMagicKP = new LoggedTunableNumber("Arm/MotionMagicKP", 30.0);
    private static final LoggedTunableNumber motionMagicKD = new LoggedTunableNumber("Arm/MotionMagicKD", 1.0);
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Arm/PositionKP", 0.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Arm/PositionKD", 0.0);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Arm/VelocityKP", 0.0);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Arm/VelocityKD", 0.0);
    private static final LoggedTunableBoolean useMotionMagic = new LoggedTunableBoolean("Arm/UseMotionMagic", true);

    protected final CANdi candi;
    protected final CANcoder cancoder;
    protected final TalonFX motor;
    private final CANdiConfiguration candiConfig;
    private final CANcoderConfiguration cancoderConfig;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> angleSignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;

    private final VoltageOut voltageControlRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage motionMagicControlRequest = new MotionMagicVoltage(0.0).withSlot(0);
    private final PositionVoltage positionControlRequest = new PositionVoltage(0.0).withSlot(1);
    private final VelocityVoltage velocityControlRequest = new VelocityVoltage(0.0).withSlot(2);

    public ArmIOHardware() {
        Optional<StatusSignal<Angle>> angleSignal = Optional.empty();
        var feedbackConfigs = new FeedbackConfigs();
        if (feedbackSensorSource == FeedbackSensorSourceValue.FusedCANcoder
                || feedbackSensorSource == FeedbackSensorSourceValue.SyncCANcoder
                || feedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder) {
            cancoderConfig = new CANcoderConfiguration()
                    .withMagnetSensor(new MagnetSensorConfigs()
                            .withMagnetOffset(Radians.zero())
                            .withSensorDirection(cancoderSensorDirection)
                            .withMagnetOffset(Radians.of(encoderOffset.getAsDouble())));
            cancoder = new CANcoder(10, Constants.canivoreBusName);
            tryUntilOk(() -> cancoder.getConfigurator().apply(cancoderConfig));

            feedbackConfigs
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(reduction)
                    .withFeedbackSensorSource(feedbackSensorSource)
                    .withFeedbackRemoteSensorID(cancoder.getDeviceID());

            angleSignal = Optional.of(cancoder.getAbsolutePosition());

            candi = null;
            candiConfig = null;
        } else if (RobotBase.isReal()
                && (feedbackSensorSource == FeedbackSensorSourceValue.FusedCANdiPWM1
                        || feedbackSensorSource == FeedbackSensorSourceValue.SyncCANdiPWM1
                        || feedbackSensorSource == FeedbackSensorSourceValue.RemoteCANdiPWM1)) {
            candiConfig = new CANdiConfiguration()
                    .withPWM1(new PWM1Configs().withAbsoluteSensorOffset(Radians.of(encoderOffset.getAsDouble())));
            candi = new CANdi(1, Constants.canivoreBusName);
            tryUntilOk(() -> candi.getConfigurator().apply(candiConfig));

            feedbackConfigs
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(reduction)
                    .withFeedbackSensorSource(feedbackSensorSource)
                    .withFeedbackRemoteSensorID(candi.getDeviceID());

            angleSignal = Optional.of(candi.getPWM1Position());

            cancoder = null;
            cancoderConfig = null;
        } else {
            feedbackConfigs.withSensorToMechanismRatio(reduction);

            cancoder = null;
            cancoderConfig = null;
            candi = null;
            candiConfig = null;
        }

        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(motorInvertedValue))
                .withFeedback(feedbackConfigs)
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(motionMagicMaxVelocity.getAsDouble())
                        .withMotionMagicAcceleration(motionMagicMaxAcceleration.getAsDouble())
                        .withMotionMagicJerk(motionMagicMaxJerk.getAsDouble())
                        .withMotionMagicExpo_kV(motionMagicKV.getAsDouble())
                        .withMotionMagicExpo_kA(motionMagicKA.getAsDouble()))
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
                        .withPeakForwardTorqueCurrent(Amps.of(100))
                        .withPeakReverseTorqueCurrent(Amps.of(100)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(100))
                        .withSupplyCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLowerLimit(40));

        motor = new TalonFX(22, Constants.canivoreBusName);
        tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));

        this.angleSignal = angleSignal.orElse(motor.getPosition());
        positionSignal = motor.getRotorPosition();
        velocitySignal = motor.getRotorVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        closedLoopReferenceSignal = motor.getClosedLoopReference();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                this.angleSignal,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                closedLoopReferenceSignal);
        motor.optimizeBusUtilization();

        if (cancoder != null) {
            cancoder.optimizeBusUtilization();
        }

        if (candi != null) {
            candi.optimizeBusUtilization();
        }
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
        inputs.rotorPosition = positionSignal.getValue();
        inputs.rotorVelocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.statorCurrent = statorCurrentSignal.getValue();
        inputs.closedLoopReference = Units.rotationsToRadians(closedLoopReferenceSignal.getValue());

        LoggedTunableNumber.ifChanged(
                0,
                () -> {
                    if (candi != null) {
                        candiConfig.PWM1.withAbsoluteSensorOffset(Radians.of(encoderOffset.getAsDouble()));
                        tryUntilOk(() -> candi.getConfigurator().apply(candiConfig));
                    }

                    if (cancoder != null) {
                        cancoderConfig.MagnetSensor.withMagnetOffset(Radians.of(encoderOffset.getAsDouble()));
                        tryUntilOk(() -> cancoder.getConfigurator().apply(cancoderConfig));
                    }
                },
                encoderOffset);

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
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageControlRequest.withOutput(voltage));
    }

    @Override
    public void setAngle(Angle angle) {
        if (useMotionMagic.get()) {
            motor.setControl(motionMagicControlRequest.withPosition(angle));
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
