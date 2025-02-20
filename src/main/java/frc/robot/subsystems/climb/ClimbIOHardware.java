package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import frc.robot.util.tuning.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ClimbIOHardware implements ClimbIO {
    private static final double kT = DCMotor.getKrakenX60(numMotors).withReduction(reduction).KtNMPerAmp * numMotors;
    private static final LoggedTunableNumber climbedPostion = new LoggedTunableNumber("Climb/ClimbedPosition", 0.45);
    private static final LoggedTunableNumber positionKP = new LoggedTunableNumber("Elevator/PositionKP", 0.0);
    private static final LoggedTunableNumber positionKD = new LoggedTunableNumber("Elevator/PositionKD", 0.0);

    protected final TalonFX motor;
    private final Servo brakeServo = new Servo(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;

    private final VoltageOut voltageControlRequest = new VoltageOut(0.0);
    private final TorqueCurrentFOC torqueCurrentControlRequest = new TorqueCurrentFOC(0.0);

    public ClimbIOHardware() {
        var motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction))
                .withSlot0(new Slot0Configs()
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(positionKP.get())
                        .withKD(positionKD.get()))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Amps.of(120))
                        .withPeakReverseTorqueCurrent(Amps.of(120)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withSupplyCurrentLimit(Amps.of(80))
                        .withSupplyCurrentLowerLimit(Amps.of(40)));

        motor = new TalonFX(35);
        tryUntilOk(() -> motor.getConfigurator().apply(motorConfig));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        torqueCurrentSignal = motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.mainLoopFrequency,
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                torqueCurrentSignal);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {}

    @Override
    public void setBrakeServoAngle(Angle angle) {}

    @Override
    public void setVoltage(Voltage voltage) {}

    @Override
    public void setTorqueCurrent(Current current) {}

    @Override
    public void setPosition(Angle position, Torque feedforward) {}

    @Override
    public void zeroPosition() {}

    @Override
    public void stop() {}
}
