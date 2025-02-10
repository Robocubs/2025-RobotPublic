package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.CustomDCMotor;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ArmIOHardware implements ArmIO {
    private static final double kT =
            CustomDCMotor.getKrakenX44(numMotors).withReduction(reduction).KtNMPerAmp * numMotors;
    private static final LoggedTunableNumber kG =
            new LoggedTunableNumber("Arm/KG", 9.81 * mass.in(Kilograms) * cg.in(Meters) / kT);
    // a * kA = i = T / kT = I * (2 pi a) / kT // kA = I * 2 pi / kT
    private static final LoggedTunableNumber positionKA =
            new LoggedTunableNumber("Arm/PositionKA", 2 * Math.PI * moi.in(KilogramSquareMeters) / kT);
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Arm/PositionKP", 500.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Arm/PositionKD", 70.0);
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
    private final StatusSignal<Current> torqueCurrentSignal;

    private final MotionMagicTorqueCurrentFOC motionMagicControlRequest =
            new MotionMagicTorqueCurrentFOC(0.0).withSlot(0);
    private final PositionTorqueCurrentFOC positionControlRequest = new PositionTorqueCurrentFOC(0.0).withSlot(1);
    private final VelocityTorqueCurrentFOC velocityControlRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(2);

    public ArmIOHardware() {
        cancoder = new CANcoder(10);
        cancoder.getConfigurator()
                .apply(new CANcoderConfiguration()
                        .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(Radians.zero())));

        motorConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(1.0)
                        .withRotorToSensorRatio(reduction)
                        .withFusedCANcoder(cancoder))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(maximumVelocity)
                        .withMotionMagicAcceleration(maximumAcceleration)
                        .withMotionMagicJerk(maximumJerk))
                .withSlot0(new Slot0Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKS(0)
                        .withKV(0)
                        .withKA(positionKA.get())
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
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
                        .withPeakForwardTorqueCurrent(Amps.of(80))
                        .withPeakReverseTorqueCurrent(Amps.of(80)));

        motor = new TalonFX(22);
        motor.getConfigurator().apply(motorConfig);

        angleSignal = cancoder.getAbsolutePosition();
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        torqueCurrentSignal = motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopPeriod.in(Milliseconds),
                angleSignal,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                torqueCurrentSignal);
        cancoder.optimizeBusUtilization();
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                angleSignal, positionSignal, velocitySignal, voltageSignal, supplyCurrentSignal, torqueCurrentSignal);
        inputs.angle = angleSignal.getValue();
        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.torqueCurrent = torqueCurrentSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.Slot0.kG = kG.get();
                    motorConfig.Slot0.kA = positionKA.get();
                    motorConfig.Slot0.kP = positionKP.get();
                    motorConfig.Slot0.kD = positionKD.get();
                    motorConfig.Slot1.kG = kG.get();
                    motorConfig.Slot1.kP = positionKP.get();
                    motorConfig.Slot1.kD = positionKD.get();
                    motorConfig.Slot2.kG = kG.get();
                    motorConfig.Slot2.kP = velocityKP.get();
                    motorConfig.Slot2.kD = velocityKD.get();
                    tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));
                },
                kG,
                positionKA,
                positionKP,
                positionKD,
                velocityKP,
                velocityKD);
    }

    @Override
    public void setAngle(Angle angle) {
        if (useMotionMagic.get()) {
            motor.setControl(motionMagicControlRequest.withPosition(angle));
        } else {
            setAngle(angle, NewtonMeters.zero());
        }
    }

    @Override
    public void setAngle(Angle angle, Torque feedforward) {
        motor.setControl(positionControlRequest.withPosition(angle).withFeedForward(feedforward.in(NewtonMeters) / kT));
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
}
