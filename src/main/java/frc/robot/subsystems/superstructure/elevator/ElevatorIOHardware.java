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
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
    private static final LoggedTunableNumber kG = new LoggedTunableNumber(
            "Elevator/KG",
            9.81 * loadMass.in(Kilograms) * sprocketRadius.in(Meters) / kT * Math.sin(elevatorAngle.in(Radians)));
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Elevator/PositionKP", 500.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Elevator/PositionKD", 40.0);
    private static final LoggedTunableNumber velocityKP = new LoggedTunableNumber("Elevator/VelocityKP", 0.0);
    private static final LoggedTunableNumber velocityKD = new LoggedTunableNumber("Elevator/VelocityKD", 0.0);
    private static final LoggedTunableBoolean useMotionMagic =
            new LoggedTunableBoolean("Elevator/UseMotionMagic", false);

    protected final TalonFX masterMotor;
    protected final TalonFX followerMotor;
    private final TalonFXConfiguration motorConfig;
    private final StatusSignal<Angle> masterAngleSignal;
    private final StatusSignal<AngularVelocity> masterVelocitySignal;
    private final StatusSignal<Voltage> masterVoltageSignal;
    private final StatusSignal<Current> masterSupplyCurrentSignal;
    private final StatusSignal<Current> masterTorqueCurrentSignal;
    private final StatusSignal<Angle> followerAngleSignal;
    private final StatusSignal<AngularVelocity> followerVelocitySignal;
    private final StatusSignal<Voltage> followerVoltageSignal;
    private final StatusSignal<Current> followerSupplyCurrentSignal;
    private final StatusSignal<Current> followerTorqueCurrentSignal;

    private final MotionMagicTorqueCurrentFOC motionMagicControlRequest =
            new MotionMagicTorqueCurrentFOC(0.0).withSlot(0);
    private final PositionTorqueCurrentFOC positionControlRequest = new PositionTorqueCurrentFOC(0.0).withSlot(1);
    private final VelocityTorqueCurrentFOC velocityControlRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(2);

    public ElevatorIOHardware() {
        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(toMotorVelocity(maximumVelocity))
                        .withMotionMagicAcceleration(RadiansPerSecondPerSecond.of(
                                maximumAcceleration.in(MetersPerSecondPerSecond) / sprocketRadius.in(Meters)))
                        .withMotionMagicJerk(RadiansPerSecondPerSecond.per(Second)
                                .of(maximumJerk.in(MetersPerSecondPerSecond.per(Second)) / sprocketRadius.in(Meters)))
                        .withMotionMagicExpo_kV(3.56)
                        .withMotionMagicExpo_kA(0.05))
                .withSlot0(new Slot0Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
                .withSlot1(new Slot1Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
                .withSlot2(new Slot2Configs()
                        .withKG(kG.get())
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(velocityKP.get())
                        .withKD(velocityKD.get()))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(toMotorPosition(maximumHeight)))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Amps.of(120))
                        .withPeakForwardTorqueCurrent(Amps.of(120)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLimit(Amps.of(40)));

        masterMotor = new TalonFX(20);
        tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));

        followerMotor = new TalonFX(21);
        tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));
        tryUntilOk(() -> followerMotor.setControl(new Follower(masterMotor.getDeviceID(), false)));

        masterAngleSignal = masterMotor.getPosition();
        masterVelocitySignal = masterMotor.getVelocity();
        masterVoltageSignal = masterMotor.getMotorVoltage();
        masterSupplyCurrentSignal = masterMotor.getSupplyCurrent();
        masterTorqueCurrentSignal = masterMotor.getTorqueCurrent();
        followerAngleSignal = followerMotor.getPosition();
        followerVelocitySignal = followerMotor.getVelocity();
        followerVoltageSignal = followerMotor.getMotorVoltage();
        followerSupplyCurrentSignal = followerMotor.getSupplyCurrent();
        followerTorqueCurrentSignal = followerMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopPeriod.in(Milliseconds),
                masterAngleSignal,
                masterVelocitySignal,
                masterVoltageSignal,
                masterSupplyCurrentSignal,
                masterTorqueCurrentSignal,
                followerAngleSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerSupplyCurrentSignal,
                followerTorqueCurrentSignal);
        masterMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                masterAngleSignal,
                masterVelocitySignal,
                masterVoltageSignal,
                masterSupplyCurrentSignal,
                masterTorqueCurrentSignal,
                followerAngleSignal,
                followerVelocitySignal,
                followerVoltageSignal,
                followerSupplyCurrentSignal,
                followerTorqueCurrentSignal);
        inputs.masterPosition = fromMotorPosition(masterAngleSignal.getValue());
        inputs.masterVelocity = fromMotorVelocity(masterVelocitySignal.getValue());
        inputs.masterVoltage = masterVoltageSignal.getValue();
        inputs.masterSupplyCurrent = masterSupplyCurrentSignal.getValue();
        inputs.masterTorqueCurrent = masterTorqueCurrentSignal.getValue();
        inputs.followerPosition = fromMotorPosition(followerAngleSignal.getValue());
        inputs.followerVelocity = fromMotorVelocity(followerVelocitySignal.getValue());
        inputs.followerVoltage = followerVoltageSignal.getValue();
        inputs.followerSupplyCurrent = followerSupplyCurrentSignal.getValue();
        inputs.followerTorqueCurrent = followerTorqueCurrentSignal.getValue();

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    motorConfig.Slot0.kG = kG.get();
                    motorConfig.Slot0.kP = positionKP.get();
                    motorConfig.Slot0.kD = positionKD.get();
                    motorConfig.Slot1.kG = kG.get();
                    motorConfig.Slot1.kP = positionKP.get();
                    motorConfig.Slot1.kD = positionKD.get();
                    motorConfig.Slot2.kG = kG.get();
                    motorConfig.Slot2.kP = velocityKP.get();
                    motorConfig.Slot2.kD = velocityKD.get();
                    tryUntilOk(() -> masterMotor.getConfigurator().apply(motorConfig));
                    tryUntilOk(() -> followerMotor.getConfigurator().apply(motorConfig));
                },
                kG,
                positionKP,
                positionKD,
                velocityKP,
                velocityKD);
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
    public void stop() {
        masterMotor.stopMotor();
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
