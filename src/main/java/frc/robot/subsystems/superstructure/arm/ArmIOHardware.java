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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

public class ArmIOHardware implements ArmIO {
    // TODO: update the motor when the library has the Kraken X44
    public static final double kT = DCMotor.getKrakenX60(1).KtNMPerAmp * reduction;
    public static final double kG = 9.81 * mass.in(Kilograms) * cg.in(Meters) / kT;

    protected final CANcoder cancoder;
    protected final TalonFX motor;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Angle> angleSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> motorCurrentSignal;

    private final MotionMagicTorqueCurrentFOC motionMagicControlRequest =
            new MotionMagicTorqueCurrentFOC(0.0).withSlot(4);
    private final PositionTorqueCurrentFOC positionControlRequest = new PositionTorqueCurrentFOC(0.0).withSlot(5);
    private final VelocityTorqueCurrentFOC velocityControlRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(6);

    public ArmIOHardware() {
        cancoder = new CANcoder(10);
        cancoder.getConfigurator()
                .apply(new CANcoderConfiguration()
                        .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(Radians.zero())));

        motor = new TalonFX(22);
        motor.getConfigurator()
                .apply(new TalonFXConfiguration()
                        .withFeedback(new FeedbackConfigs()
                                .withSensorToMechanismRatio(1.0)
                                .withRotorToSensorRatio(reduction)
                                .withFusedCANcoder(cancoder))
                        .withMotionMagic(new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(maximumVelocity)
                                .withMotionMagicAcceleration(maximumAcceleration)
                                .withMotionMagicJerk(maximumJerk))
                        .withSlot0(new Slot0Configs()
                                .withKG(kG)
                                .withGravityType(GravityTypeValue.Arm_Cosine)
                                .withKS(0)
                                .withKV(0)
                                .withKA(2 * Math.PI * moi.in(KilogramSquareMeters) / kT)
                                .withKP(100.0)
                                .withKD(60.0))
                        .withSlot1(new Slot1Configs()
                                .withKG(kG)
                                .withGravityType(GravityTypeValue.Arm_Cosine)
                                .withKP(100.0)
                                .withKD(1.0))
                        .withSlot2(new Slot2Configs()
                                .withKG(kG)
                                .withGravityType(GravityTypeValue.Arm_Cosine)
                                .withKP(0.0)
                                .withKD(0.0))
                        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(maximumAngle)
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(minimumAngle))
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                .withPeakForwardTorqueCurrent(Amps.of(80))
                                .withPeakForwardTorqueCurrent(Amps.of(80))));

        positionSignal = motor.getPosition();
        angleSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        motorCurrentSignal = motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopPeriod.in(Milliseconds),
                angleSignal,
                positionSignal,
                velocitySignal,
                motorCurrentSignal);
        cancoder.optimizeBusUtilization();
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(angleSignal, positionSignal, velocitySignal, motorCurrentSignal);
        inputs.angle = angleSignal.getValue();
        inputs.position = positionSignal.getValue();
        inputs.velocity = velocitySignal.getValue();
        inputs.current = motorCurrentSignal.getValue();
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setControl(motionMagicControlRequest.withPosition(angle));
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
