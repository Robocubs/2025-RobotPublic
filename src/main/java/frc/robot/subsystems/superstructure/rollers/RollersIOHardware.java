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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.rollers.RollersConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class RollersIOHardware implements RollersIO {
    private static final LoggedTunableNumber coralPositionKP = new LoggedTunableNumber("Rollers/CoralPositionKP", 0.0);
    private static final LoggedTunableNumber coralPositionKD = new LoggedTunableNumber("Rollers/CoralPositionKD", 0.0);
    private static final LoggedTunableNumber coralVelocityKP = new LoggedTunableNumber("Rollers/CoralVelocityKP", 0.0);
    private static final LoggedTunableNumber coralVelocityKD = new LoggedTunableNumber("Rollers/CoralVelocityKD", 0.0);
    private static final LoggedTunableNumber hybridPositionKP =
            new LoggedTunableNumber("Rollers/HybridPositionKP", 0.0);
    private static final LoggedTunableNumber hybridPositionKD =
            new LoggedTunableNumber("Rollers/HybridPositionKD", 0.0);
    private static final LoggedTunableNumber hybridVelocityKP =
            new LoggedTunableNumber("Rollers/HybridVelocityKP", 0.0);
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
    private final StatusSignal<Angle> hybridPositionSignal;
    private final StatusSignal<AngularVelocity> hybridVelocitySignal;
    private final StatusSignal<Voltage> hybridVoltageSignal;
    private final StatusSignal<Current> hybridSupplyCurrentSignal;
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
                .withSlot0(
                        new Slot0Configs().withKP(coralPositionKP.getAsDouble()).withKD(coralPositionKD.getAsDouble()))
                .withSlot1(
                        new Slot1Configs().withKP(coralVelocityKP.getAsDouble()).withKD(coralVelocityKD.getAsDouble()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(30)));
        coralMotor = new TalonFX(33, Constants.canivoreBusName);
        tryUntilOk(() -> coralMotor.getConfigurator().apply(coralMotorConfig));

        hybridMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted((InvertedValue.CounterClockwise_Positive)))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(hybridRollerReduction))
                .withSlot0(new Slot0Configs()
                        .withKP(hybridPositionKP.getAsDouble())
                        .withKD(hybridPositionKD.getAsDouble()))
                .withSlot1(new Slot1Configs()
                        .withKP(hybridVelocityKP.getAsDouble())
                        .withKD(hybridVelocityKD.getAsDouble()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(30)));
        hybridMotor = new TalonFX(34, Constants.canivoreBusName);
        tryUntilOk(() -> hybridMotor.getConfigurator().apply(hybridMotorConfig));

        coralCanrange = new CANrange(0, Constants.canivoreBusName);
        algaeCanrange = new CANrange(1, Constants.canivoreBusName);

        coralPositionSignal = coralMotor.getPosition();
        coralVelocitySignal = coralMotor.getVelocity();
        coralVoltageSignal = coralMotor.getMotorVoltage();
        coralSupplyCurrentSignal = coralMotor.getSupplyCurrent();
        hybridPositionSignal = hybridMotor.getPosition();
        hybridVelocitySignal = hybridMotor.getVelocity();
        hybridVoltageSignal = hybridMotor.getMotorVoltage();
        hybridSupplyCurrentSignal = hybridMotor.getSupplyCurrent();
        coralDistanceSignal = coralCanrange.getDistance();
        algaeDistanceSignal = algaeCanrange.getDistance();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                coralPositionSignal,
                coralVelocitySignal,
                coralVoltageSignal,
                coralSupplyCurrentSignal,
                hybridPositionSignal,
                hybridVelocitySignal,
                hybridVoltageSignal,
                hybridSupplyCurrentSignal,
                coralDistanceSignal,
                algaeDistanceSignal);
        coralMotor.optimizeBusUtilization();
        hybridMotor.optimizeBusUtilization();
        coralCanrange.optimizeBusUtilization();
        algaeCanrange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        /*
         * TODO: Implement
         * 1. Refresh input signals
         * 2. Update input fields
         * 3. Check for changes to tunable numbers
         */
    }

    @Override
    public void setCoralVelocity(AngularVelocity velocity) {}

    @Override
    public void setCoralPosition(Angle position) {}

    @Override
    public void stopCoral() {}

    @Override
    public void setHybridVelocity(AngularVelocity velocity) {}

    @Override
    public void setHybridPosition(Angle position) {}

    @Override
    public void stopHybrid() {}
}
