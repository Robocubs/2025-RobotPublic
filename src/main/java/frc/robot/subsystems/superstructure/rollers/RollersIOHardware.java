package frc.robot.subsystems.superstructure.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
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
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Rollers/PositionKP", 5.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Rollers/PositionKD", 0.0);
    private static final LoggedTunableNumber velocityKV = new LoggedTunableNumber("Rollers/VelocityKV", 1.0);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Rollers/VelocityKP", 3.0);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Rollers/VelocityKD", 0.0);

    protected final TalonFX motor;
    protected final CANrange coralCanrange;
    protected final CANrange algaeCanrange;
    protected final CANrange elevatorCanrange;
    protected final CANrange funnelCanrange;
    private final TalonFXConfiguration coralMotorConfig;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;
    private final StatusSignal<Distance> coralDistanceSignal;
    private final StatusSignal<Double> coralSignalStrengthSignal;
    private final StatusSignal<Distance> algaeDistanceSignal;
    private final StatusSignal<Double> algaeSignalStrengthSignal;
    private final StatusSignal<Distance> elevatorDistanceSignal;
    private final StatusSignal<Double> elevatorSignalStrengthSignal;
    private final StatusSignal<Distance> funnelDistanceSignal;
    private final StatusSignal<Double> funnelSignalStrengthSignal;

    private final PositionVoltage coralPositionControlRequest = new PositionVoltage(0.0).withSlot(0);
    private final VelocityVoltage coralVelocityControlRequest = new VelocityVoltage(0.0).withSlot(1);

    public RollersIOHardware() {
        coralMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted((InvertedValue.Clockwise_Positive)))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction))
                .withSlot0(new Slot0Configs().withKP(positionKP.get()).withKD(positionKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKV(velocityKV.get())
                        .withKP(velocityKP.get())
                        .withKD(velocityKD.get()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLowerLimit(30));
        motor = new TalonFX(34, Constants.rioBusName);
        tryUntilOk(() -> motor.getConfigurator().apply(coralMotorConfig));

        coralCanrange = new CANrange(3, Constants.rioBusName);
        algaeCanrange = new CANrange(2, Constants.rioBusName);
        elevatorCanrange = new CANrange(1, Constants.rioBusName);
        funnelCanrange = new CANrange(4, Constants.rioBusName);

        var coralCanrageConfig = new CANrangeConfiguration()
                .withFovParams(new FovParamsConfigs().withFOVRangeX(6.75).withFOVRangeY(6.75));
        tryUntilOk(() -> coralCanrange.getConfigurator().apply(coralCanrageConfig));

        var algaeCanrageConfig = new CANrangeConfiguration()
                .withFovParams(new FovParamsConfigs().withFOVRangeX(6.75).withFOVRangeY(6.75));
        tryUntilOk(() -> algaeCanrange.getConfigurator().apply(algaeCanrageConfig));

        var elevatorCanrangeConfig = new CANrangeConfiguration()
                .withFovParams(new FovParamsConfigs().withFOVRangeX(6.75).withFOVRangeY(15));
        tryUntilOk(() -> elevatorCanrange.getConfigurator().apply(elevatorCanrangeConfig));

        var funnelCanrangeConfig = new CANrangeConfiguration()
                .withFovParams(new FovParamsConfigs().withFOVRangeX(6.75).withFOVRangeY(15));
        tryUntilOk(() -> funnelCanrange.getConfigurator().apply(funnelCanrangeConfig));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        closedLoopReferenceSignal = motor.getClosedLoopReference();
        coralDistanceSignal = coralCanrange.getDistance();
        coralSignalStrengthSignal = coralCanrange.getSignalStrength();
        algaeDistanceSignal = algaeCanrange.getDistance();
        algaeSignalStrengthSignal = funnelCanrange.getSignalStrength();
        elevatorDistanceSignal = elevatorCanrange.getDistance();
        elevatorSignalStrengthSignal = elevatorCanrange.getSignalStrength();
        funnelDistanceSignal = funnelCanrange.getDistance();
        funnelSignalStrengthSignal = funnelCanrange.getSignalStrength();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                closedLoopReferenceSignal,
                coralDistanceSignal,
                algaeDistanceSignal,
                elevatorDistanceSignal,
                elevatorSignalStrengthSignal,
                funnelDistanceSignal,
                funnelSignalStrengthSignal);
        motor.optimizeBusUtilization();
        coralCanrange.optimizeBusUtilization();
        algaeCanrange.optimizeBusUtilization();
        elevatorCanrange.optimizeBusUtilization();
        funnelCanrange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                closedLoopReferenceSignal,
                coralDistanceSignal,
                coralSignalStrengthSignal,
                algaeDistanceSignal,
                algaeSignalStrengthSignal,
                elevatorDistanceSignal,
                elevatorSignalStrengthSignal,
                funnelDistanceSignal,
                funnelSignalStrengthSignal);

        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.voltage = voltageSignal.getValue();
        inputs.supplyCurrent = supplyCurrentSignal.getValue();
        inputs.closedLoopReferenceSignal = Units.rotationsToRadians(closedLoopReferenceSignal.getValue());
        inputs.coralDetectorDistance = coralDistanceSignal.getValue();
        inputs.coralSignalStrengthSignal = coralSignalStrengthSignal.getValue();
        inputs.algaeDetectorDistance = algaeDistanceSignal.getValue();
        inputs.algaeSignalStrengthSignal = algaeSignalStrengthSignal.getValue();
        inputs.elevatorDetectorDistance = elevatorDistanceSignal.getValue();
        inputs.elevatorSignalStrength = elevatorSignalStrengthSignal.getValue();
        inputs.funnelDetectorDistance = funnelDistanceSignal.getValue();
        inputs.funnelSignalStrength = funnelSignalStrengthSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    coralMotorConfig.Slot0.kP = positionKP.get();
                    coralMotorConfig.Slot0.kD = positionKD.get();
                    coralMotorConfig.Slot1.kV = velocityKV.get();
                    coralMotorConfig.Slot1.kP = velocityKP.get();
                    coralMotorConfig.Slot1.kD = velocityKD.get();
                    tryUntilOk(() -> motor.getConfigurator().apply(coralMotorConfig));
                },
                positionKP,
                positionKD,
                velocityKV,
                velocityKP,
                velocityKD);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(coralVelocityControlRequest.withVelocity(velocity));
    }

    @Override
    public void setPosition(Angle position) {
        motor.setControl(coralPositionControlRequest.withPosition(position));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
