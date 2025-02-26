package frc.robot.subsystems.superstructure.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class RollersIOHardware implements RollersIO {
    private static final LoggedTunableNumber coralPositionKP = new LoggedTunableNumber("Rollers/CoralPositionKP", 5.0);
    private static final LoggedTunableNumber coralPositionKD = new LoggedTunableNumber("Rollers/CoralPositionKD", 0.0);
    private static final LoggedTunableNumber coralVelocityKV = new LoggedTunableNumber("Rollers/CoralVelocityKV", 1.0);
    private static final LoggedTunableNumber coralVelocityKP = new LoggedTunableNumber("Rollers/CoralVelocityKP", 3.0);
    private static final LoggedTunableNumber coralVelocityKD = new LoggedTunableNumber("Rollers/CoralVelocityKD", 0.0);
    private static final LoggedTunableNumber hybridPositionKP =
            new LoggedTunableNumber("Rollers/HybridPositionKP", 5.0);
    private static final LoggedTunableNumber hybridPositionKD =
            new LoggedTunableNumber("Rollers/HybridPositionKD", 0.0);
    private static final LoggedTunableNumber hybridVelocityKV =
            new LoggedTunableNumber("Rollers/HybridVelocityKV", 0.7);
    private static final LoggedTunableNumber hybridVelocityKP =
            new LoggedTunableNumber("Rollers/HybridVelocityKP", 1.0);
    private static final LoggedTunableNumber hybridVelocityKD =
            new LoggedTunableNumber("Rollers/HybridVelocityKD", 0.0);

    protected final TalonFX coralMotor;
    protected final TalonFX hybridMotor;
    protected final CANrange coralCanrange;
    protected final CANrange algaeCanrange;
    private final TalonFXConfiguration coralMotorConfig;
    private final TalonFXConfiguration hybridMotorConfig;

    private final StatusSignal<Angle> coralPositionSignal;
    private final StatusSignal<AngularVelocity> coralVelocitySignal;
    private final StatusSignal<Voltage> coralVoltageSignal;
    private final StatusSignal<Current> coralSupplyCurrentSignal;
    private final StatusSignal<Double> coralClosedLoopReferenceSignal;
    private final StatusSignal<Angle> hybridPositionSignal;
    private final StatusSignal<AngularVelocity> hybridVelocitySignal;
    private final StatusSignal<Voltage> hybridVoltageSignal;
    private final StatusSignal<Current> hybridSupplyCurrentSignal;
    private final StatusSignal<Double> hybridClosedLoopReferenceSignal;
    private final StatusSignal<Distance> coralDistanceSignal;
    private final StatusSignal<Distance> algaeDistanceSignal;

    private final PositionVoltage coralPositionControlRequest = new PositionVoltage(0.0).withSlot(0);
    private final VelocityVoltage coralVelocityControlRequest = new VelocityVoltage(0.0).withSlot(1);
    private final PositionVoltage hybridPositionControlRequest = new PositionVoltage(0.0).withSlot(0);
    private final VelocityVoltage hybridVelocityControlRequest = new VelocityVoltage(0.0).withSlot(1);

    public RollersIOHardware() {
        coralMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted((InvertedValue.CounterClockwise_Positive)))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(coralRollerReduction))
                .withSlot0(new Slot0Configs().withKP(coralPositionKP.get()).withKD(coralPositionKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKV(coralVelocityKV.get())
                        .withKP(coralVelocityKP.get())
                        .withKD(coralVelocityKD.get()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLowerLimit(30));
        coralMotor = new TalonFX(34, Constants.canivoreBusName);
        tryUntilOk(() -> coralMotor.getConfigurator().apply(coralMotorConfig));

        hybridMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted((InvertedValue.Clockwise_Positive)))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(hybridRollerReduction))
                .withSlot0(new Slot0Configs().withKP(hybridPositionKP.get()).withKD(hybridPositionKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKV(hybridVelocityKV.get())
                        .withKP(hybridVelocityKP.get())
                        .withKD(hybridVelocityKD.get()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLowerLimit(30));
        hybridMotor = new TalonFX(33, Constants.canivoreBusName);
        tryUntilOk(() -> hybridMotor.getConfigurator().apply(hybridMotorConfig));

        coralCanrange = new CANrange(3, Constants.canivoreBusName);
        algaeCanrange = new CANrange(2, Constants.canivoreBusName);

        coralPositionSignal = coralMotor.getPosition();
        coralVelocitySignal = coralMotor.getVelocity();
        coralVoltageSignal = coralMotor.getMotorVoltage();
        coralSupplyCurrentSignal = coralMotor.getSupplyCurrent();
        coralClosedLoopReferenceSignal = coralMotor.getClosedLoopReference();
        hybridPositionSignal = hybridMotor.getPosition();
        hybridVelocitySignal = hybridMotor.getVelocity();
        hybridVoltageSignal = hybridMotor.getMotorVoltage();
        hybridSupplyCurrentSignal = hybridMotor.getSupplyCurrent();
        hybridClosedLoopReferenceSignal = hybridMotor.getClosedLoopReference();
        coralDistanceSignal = coralCanrange.getDistance();
        algaeDistanceSignal = algaeCanrange.getDistance();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                coralPositionSignal,
                coralVelocitySignal,
                coralVoltageSignal,
                coralSupplyCurrentSignal,
                coralClosedLoopReferenceSignal,
                hybridPositionSignal,
                hybridVelocitySignal,
                hybridVoltageSignal,
                hybridSupplyCurrentSignal,
                hybridClosedLoopReferenceSignal,
                coralDistanceSignal,
                algaeDistanceSignal);
        coralMotor.optimizeBusUtilization();
        hybridMotor.optimizeBusUtilization();
        coralCanrange.optimizeBusUtilization();
        algaeCanrange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                coralPositionSignal,
                coralVelocitySignal,
                coralVoltageSignal,
                coralSupplyCurrentSignal,
                coralClosedLoopReferenceSignal,
                hybridPositionSignal,
                hybridVelocitySignal,
                hybridVoltageSignal,
                hybridSupplyCurrentSignal,
                hybridClosedLoopReferenceSignal,
                coralDistanceSignal,
                algaeDistanceSignal);

        inputs.coralPosition = coralPositionSignal.getValue();
        inputs.coralVelocity = coralVelocitySignal.getValue();
        inputs.coralVoltage = coralVoltageSignal.getValue();
        inputs.coralSupplyCurrent = coralSupplyCurrentSignal.getValue();
        inputs.coralClosedLoopReferenceSignal = Units.rotationsToRadians(coralClosedLoopReferenceSignal.getValue());
        inputs.hybridPosition = hybridPositionSignal.getValue();
        inputs.hybridVelocity = hybridVelocitySignal.getValue();
        inputs.hybridVoltage = hybridVoltageSignal.getValue();
        inputs.hybridSupplyCurrent = hybridSupplyCurrentSignal.getValue();
        inputs.hybridClosedLoopReferenceSignal = Units.rotationsToRadians(hybridClosedLoopReferenceSignal.getValue());
        inputs.coralDetectorDistance = coralDistanceSignal.getValue();
        inputs.algaeDetectorDistance = algaeDistanceSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    coralMotorConfig.Slot0.kP = coralPositionKP.get();
                    coralMotorConfig.Slot0.kD = coralPositionKD.get();
                    coralMotorConfig.Slot1.kV = coralVelocityKV.get();
                    coralMotorConfig.Slot1.kP = coralVelocityKP.get();
                    coralMotorConfig.Slot1.kD = coralVelocityKD.get();
                    tryUntilOk(() -> coralMotor.getConfigurator().apply(coralMotorConfig));
                },
                coralPositionKP,
                coralPositionKD,
                coralVelocityKV,
                coralVelocityKP,
                coralVelocityKD);

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    hybridMotorConfig.Slot0.kP = hybridPositionKP.get();
                    hybridMotorConfig.Slot0.kD = hybridPositionKD.get();
                    hybridMotorConfig.Slot1.kV = hybridVelocityKV.get();
                    hybridMotorConfig.Slot1.kP = hybridVelocityKP.get();
                    hybridMotorConfig.Slot1.kD = hybridVelocityKD.get();
                    tryUntilOk(() -> hybridMotor.getConfigurator().apply(hybridMotorConfig));
                },
                hybridPositionKP,
                hybridPositionKD,
                hybridVelocityKV,
                hybridVelocityKP,
                hybridVelocityKD);
    }

    @Override
    public void setCoralVelocity(AngularVelocity velocity) {
        coralMotor.setControl(coralVelocityControlRequest.withVelocity(velocity));
    }

    @Override
    public void setCoralPosition(Angle position) {
        coralMotor.setControl(coralPositionControlRequest.withPosition(position));
    }

    @Override
    public void stopCoral() {
        coralMotor.stopMotor();
    }

    @Override
    public void setHybridVelocity(AngularVelocity velocity) {
        hybridMotor.setControl(hybridVelocityControlRequest.withVelocity(velocity));
    }

    @Override
    public void setHybridPosition(Angle position) {
        hybridMotor.setControl(hybridPositionControlRequest.withPosition(position));
    }

    @Override
    public void stopHybrid() {
        hybridMotor.stopMotor();
    }
}
