package frc.robot.subsystems.superstructure.funnel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.superstructure.funnel.FunnelConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class FunnelIOHardware implements FunnelIO {
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Funnel/PositionKP", 0.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Funnel/PositionKD", 0.0);
    private static final LoggedTunableNumber velocityKV = new LoggedTunableNumber("Funnel/VelocityKV", 1.3);
    private static final LoggedTunableNumber velocityKA = new LoggedTunableNumber("Funnel/VelocityKA", 0.0);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Funnel/VelocityKP", 0.0);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Funnel/VelocityKD", 0.0);

    protected final TalonFXS masterMotor;
    protected final TalonFXS followerMotor;
    private final TalonFXSConfiguration motorConfig;

    private final StatusSignal<Angle> masterPositionSignal;
    private final StatusSignal<AngularVelocity> masterVelocitySignal;
    private final StatusSignal<Voltage> masterVoltageSignal;
    private final StatusSignal<Current> masterSupplyCurrentSignal;
    private final StatusSignal<Angle> followerPositionSignal;
    private final StatusSignal<AngularVelocity> followerVelocitySignal;
    private final StatusSignal<Voltage> followerVoltageSignal;
    private final StatusSignal<Current> followerSupplyCurrentSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;

    private final VoltageOut voltageControlRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityControlRequest = new VelocityVoltage(0.0).withSlot(0);

    public FunnelIOHardware() {
        motorConfig = new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs()
                        .withMotorArrangement(MotorArrangementValue.NEO550_JST)
                        .withAdvancedHallSupport(AdvancedHallSupportValue.Enabled))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withExternalFeedback(new ExternalFeedbackConfigs()
                        .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Commutation)
                        .withSensorToMechanismRatio(reduction)
                        .withRotorToSensorRatio(1))
                .withSlot0(new Slot0Configs()
                        .withKV(velocityKV.get())
                        .withKA(velocityKA.get())
                        .withKP(velocityKP.get())
                        .withKD(velocityKD.get()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLimit(30)
                        .withSupplyCurrentLowerLimit(Amps.of(30)));

        masterMotor = new TalonFXS(36, Constants.canivoreBusName);
        tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));

        followerMotor = new TalonFXS(37, Constants.canivoreBusName);
        tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
        // tryUntilOk(() -> followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true)));

        masterPositionSignal = masterMotor.getPosition();
        masterVelocitySignal = masterMotor.getVelocity();
        masterVoltageSignal = masterMotor.getMotorVoltage();
        masterSupplyCurrentSignal = masterMotor.getSupplyCurrent();
        followerPositionSignal = followerMotor.getPosition();
        followerVelocitySignal = followerMotor.getVelocity();
        followerVoltageSignal = followerMotor.getMotorVoltage();
        followerSupplyCurrentSignal = followerMotor.getSupplyCurrent();
        closedLoopReferenceSignal = masterMotor.getClosedLoopReference();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                masterPositionSignal,
                masterVelocitySignal,
                masterVoltageSignal,
                masterSupplyCurrentSignal,
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerSupplyCurrentSignal,
                closedLoopReferenceSignal);
        masterMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs) {

        BaseStatusSignal.refreshAll(
                masterPositionSignal,
                masterVelocitySignal,
                masterVoltageSignal,
                masterSupplyCurrentSignal,
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerSupplyCurrentSignal,
                closedLoopReferenceSignal);
        inputs.masterPosition = masterPositionSignal.getValue();
        inputs.masterVelocity = masterVelocitySignal.getValue();
        inputs.masterVoltage = masterVoltageSignal.getValue();
        inputs.masterSupplyCurrent = masterSupplyCurrentSignal.getValue();
        inputs.followerPosition = followerPositionSignal.getValue();
        inputs.followerVelocity = followerVelocitySignal.getValue();
        inputs.followerVoltage = followerVoltageSignal.getValue();
        inputs.followerSupplyCurrent = followerSupplyCurrentSignal.getValue();
        inputs.closedLoopReference = closedLoopReferenceSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.Slot0.kV = velocityKV.get();
                    motorConfig.Slot0.kA = velocityKA.get();
                    motorConfig.Slot0.kP = velocityKP.get();
                    motorConfig.Slot0.kD = velocityKD.get();
                    tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));
                    tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
                },
                positionKP,
                positionKD,
                velocityKV,
                velocityKA,
                velocityKP,
                velocityKD);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        masterMotor.setControl(voltageControlRequest.withOutput(voltage));
        followerMotor.setControl(voltageControlRequest.withOutput(voltage.times(-1)));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        masterMotor.setControl(velocityControlRequest.withVelocity(velocity));
        followerMotor.setControl(velocityControlRequest.withVelocity(velocity.times(-1)));
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        followerMotor.stopMotor();
    }
}
