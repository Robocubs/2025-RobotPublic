package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ElevatorIOHardware implements ElevatorIO {
    private static final double kT = DCMotor.getKrakenX60Foc(numMotors).withReduction(reduction).KtNMPerAmp * numMotors;
    private static final LoggedTunableNumber kGCurrent = new LoggedTunableNumber(
            "Elevator/KGCurrent",
            9.81 * loadMass.in(Kilograms) * sprocketRadius.in(Meters) / kT * Math.sin(elevatorAngle.in(Radians)));
    private static final LoggedTunableNumber kGVoltage = new LoggedTunableNumber("Elevator/KGVoltage", 0.54);
    private static final LoggedTunableNumber motionMagicExpoKV = new LoggedTunableNumber("Elevator/MotionMagicKV", 2.3);
    private static final LoggedTunableNumber motionMagicExpoKA =
            new LoggedTunableNumber("Elevator/MotionMagicKA", 0.05);
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Elevator/PositionKP", 150.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Elevator/PositionKD", 10.0);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Elevator/VelocityKP", 0.0);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Elevator/VelocityKD", 0.0);
    private static final LoggedTunableBoolean useMotionMagic =
            new LoggedTunableBoolean("Elevator/UseMotionMagic", true);

    protected final TalonFX masterMotor;
    protected final TalonFX followerMotor;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Angle> masterPositionSignal;
    private final StatusSignal<AngularVelocity> masterVelocitySignal;
    private final StatusSignal<Voltage> masterVoltageSignal;
    private final StatusSignal<Current> masterSupplyCurrentSignal;
    private final StatusSignal<Current> masterTorqueCurrentSignal;
    private final StatusSignal<Angle> followerPositionSignal;
    private final StatusSignal<AngularVelocity> followerVelocitySignal;
    private final StatusSignal<Voltage> followerVoltageSignal;
    private final StatusSignal<Current> followerSupplyCurrentSignal;
    private final StatusSignal<Current> followerTorqueCurrentSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;

    private final VoltageOut voltageControlRequest = new VoltageOut(0.0);
    private final TorqueCurrentFOC torqueCurrentControlRequest = new TorqueCurrentFOC(0.0);
    private final MotionMagicTorqueCurrentFOC motionMagicControlRequest =
            new MotionMagicTorqueCurrentFOC(0.0).withSlot(0);
    private final PositionTorqueCurrentFOC positionControlRequest = new PositionTorqueCurrentFOC(0.0).withSlot(1);
    private final VelocityVoltage velocityControlRequest = new VelocityVoltage(0.0).withSlot(2);

    public ElevatorIOHardware() {
        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(motorInvertedValue))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(toMotorVelocity(maximumVelocity))
                        .withMotionMagicAcceleration(RadiansPerSecondPerSecond.of(
                                maximumAcceleration.in(MetersPerSecondPerSecond) / sprocketRadius.in(Meters)))
                        .withMotionMagicJerk(RadiansPerSecondPerSecond.per(Second)
                                .of(maximumJerk.in(MetersPerSecondPerSecond.per(Second)) / sprocketRadius.in(Meters)))
                        .withMotionMagicExpo_kV(motionMagicExpoKV.get())
                        .withMotionMagicExpo_kA(motionMagicExpoKA.get()))
                .withSlot0(new Slot0Configs()
                        .withKG(kGCurrent.get())
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKG(kGCurrent.get())
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
                .withSlot2(new Slot2Configs()
                        .withKG(kGVoltage.get())
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(velocityKP.get())
                        .withKD(velocityKD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(toMotorPosition(maximumHeight)))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Amps.of(90))
                        .withPeakReverseTorqueCurrent(Amps.of(20)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(90))
                        .withSupplyCurrentLimit(60)
                        .withSupplyCurrentLowerLimit(Amps.of(40)));

        masterMotor = new TalonFX(30, Constants.canivoreBusName);
        tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));

        followerMotor = new TalonFX(31, Constants.canivoreBusName);
        tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
        tryUntilOk(() -> followerMotor.setControl(new Follower(masterMotor.getDeviceID(), false)));

        masterPositionSignal = masterMotor.getPosition();
        masterVelocitySignal = masterMotor.getVelocity();
        masterVoltageSignal = masterMotor.getMotorVoltage();
        masterSupplyCurrentSignal = masterMotor.getSupplyCurrent();
        masterTorqueCurrentSignal = masterMotor.getTorqueCurrent();
        followerPositionSignal = followerMotor.getPosition();
        followerVelocitySignal = followerMotor.getVelocity();
        followerVoltageSignal = followerMotor.getMotorVoltage();
        followerSupplyCurrentSignal = followerMotor.getSupplyCurrent();
        followerTorqueCurrentSignal = followerMotor.getTorqueCurrent();
        closedLoopReferenceSignal = masterMotor.getClosedLoopReference();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                masterPositionSignal,
                masterVelocitySignal,
                masterVoltageSignal,
                masterSupplyCurrentSignal,
                masterTorqueCurrentSignal,
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerSupplyCurrentSignal,
                followerTorqueCurrentSignal,
                closedLoopReferenceSignal);
        masterMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                masterPositionSignal,
                masterVelocitySignal,
                masterVoltageSignal,
                masterSupplyCurrentSignal,
                masterTorqueCurrentSignal,
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerSupplyCurrentSignal,
                followerTorqueCurrentSignal,
                closedLoopReferenceSignal);
        inputs.masterPosition = fromMotorPosition(masterPositionSignal.getValue());
        inputs.masterVelocity = fromMotorVelocity(masterVelocitySignal.getValue());
        inputs.masterVoltage = masterVoltageSignal.getValue();
        inputs.masterSupplyCurrent = masterSupplyCurrentSignal.getValue();
        inputs.masterTorqueCurrent = masterTorqueCurrentSignal.getValue();
        inputs.followerPosition = fromMotorPosition(followerPositionSignal.getValue());
        inputs.followerVelocity = fromMotorVelocity(followerVelocitySignal.getValue());
        inputs.followerVoltage = followerVoltageSignal.getValue();
        inputs.followerSupplyCurrent = followerSupplyCurrentSignal.getValue();
        inputs.followerTorqueCurrent = followerTorqueCurrentSignal.getValue();
        inputs.closedLoopReference = closedLoopReferenceSignal.getValue() * sprocketRadius.in(Meters);

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.MotionMagic.MotionMagicExpo_kV = motionMagicExpoKV.get();
                    motorConfig.MotionMagic.MotionMagicExpo_kA = motionMagicExpoKA.get();
                    motorConfig.Slot0.kG = kGCurrent.get();
                    motorConfig.Slot0.kP = positionKP.get();
                    motorConfig.Slot0.kD = positionKD.get();
                    motorConfig.Slot1.kG = kGCurrent.get();
                    motorConfig.Slot1.kP = positionKP.get();
                    motorConfig.Slot1.kD = positionKD.get();
                    motorConfig.Slot2.kG = kGVoltage.get();
                    motorConfig.Slot2.kP = velocityKP.get();
                    motorConfig.Slot2.kD = velocityKD.get();
                    tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));
                    tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
                },
                kGCurrent,
                kGVoltage,
                motionMagicExpoKV,
                motionMagicExpoKA,
                positionKP,
                positionKD,
                velocityKP,
                velocityKD);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        masterMotor.setControl(voltageControlRequest.withOutput(voltage));
    }

    @Override
    public void setTorqueCurrent(Current current) {
        masterMotor.setControl(torqueCurrentControlRequest.withOutput(current));
    }

    @Override
    public void setPosition(Distance position) {
        if (useMotionMagic.get()) {
            masterMotor.setControl(motionMagicControlRequest.withPosition(toMotorPosition(position)));
        } else {
            setPosition(position, Newtons.zero());
        }
    }

    @Override
    public void setPosition(Distance position, Force feedforward) {
        masterMotor.setControl(positionControlRequest
                .withPosition(toMotorPosition(position))
                .withFeedForward(toTorqueCurrentAmps(feedforward)));
    }

    @Override
    public void setVelocity(LinearVelocity velocity, Force feedforward) {
        masterMotor.setControl(velocityControlRequest
                .withVelocity(toMotorVelocity(velocity))
                .withFeedForward(toTorqueCurrentAmps(feedforward)));
    }

    @Override
    public void zeroPosition() {
        masterMotor.setPosition(0, .01);
        followerMotor.setPosition(0, .01);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        masterMotor.setNeutralMode(neutralMode);
        followerMotor.setNeutralMode(neutralMode);
    }

    private static Angle toMotorPosition(Distance position) {
        return Radians.of(position.in(Meters) / sprocketRadius.in(Meters));
    }

    private static AngularVelocity toMotorVelocity(LinearVelocity velocity) {
        return RadiansPerSecond.of(velocity.in(MetersPerSecond) / sprocketRadius.in(Meters));
    }

    private static double toTorqueCurrentAmps(Force force) {
        return force.in(Newtons) * sprocketRadius.in(Meters) / kT;
    }

    private static Distance fromMotorPosition(Angle position) {
        return Meters.of(position.in(Radians) * sprocketRadius.in(Meters));
    }

    private static LinearVelocity fromMotorVelocity(AngularVelocity velocity) {
        return MetersPerSecond.of(velocity.in(RadiansPerSecond) * sprocketRadius.in(Meters));
    }
}
