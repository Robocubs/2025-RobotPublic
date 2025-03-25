package frc.robot.subsystems.climb;

import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tuning.LoggedTunableMeasure;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Climb extends SubsystemBase {
    private static final LoggedTunableMeasure<AngleUnit, Angle> climbedPosition =
            new LoggedTunableMeasure<>("Climb/ClimbedPosition", Radians.of(-13));
    private static final LoggedTunableMeasure<AngleUnit, Angle> preDeployedPosition =
            new LoggedTunableMeasure<>("Climb/PreDeployedPosition", Radians.of(-18));
    private static final LoggedTunableMeasure<AngleUnit, Angle> releaseStartPosition =
            new LoggedTunableMeasure<>("Climb/ReleaseStartPosition", Radians.of(-25));
    private static final LoggedTunableMeasure<AngleUnit, Angle> releasePosition =
            new LoggedTunableMeasure<>("Climb/ReleasePosition", Radians.of(-36));
    private static final LoggedTunableMeasure<AngleUnit, Angle> deployedPosition =
            new LoggedTunableMeasure<>("Climb/DeployedPosition", Radians.of(-32));
    private static final LoggedTunableMeasure<AngleUnit, Angle> brakeEngagedAngle =
            new LoggedTunableMeasure<>("Climb/BrakeEngagedAngle", Degrees.of(135.0));
    private static final LoggedTunableMeasure<AngleUnit, Angle> brakeDisengagedAngle =
            new LoggedTunableMeasure<>("Climb/BrakeDisengagedAngle", Degrees.of(180));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> minBrakeSpeed =
            new LoggedTunableMeasure<>("Climb/MinBrakeSpeed", RadiansPerSecond.of(-0.2));
    private static final LoggedTunableMeasure<CurrentUnit, Current> zeroTorqueCurrent =
            new LoggedTunableMeasure<>("Climb/ZeroTorqueCurrent", Amps.of(5));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> zeroVelocityLimit =
            new LoggedTunableMeasure<>("Climb/ZeroVelocityLimit", RadiansPerSecond.of(0.1));
    private static final LoggedTunableMeasure<VoltageUnit, Voltage> retractVoltage =
            new LoggedTunableMeasure<>("Climb/RetractVoltage", Volts.of(10));

    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private Optional<Angle> zeroedPosition = Optional.empty();

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public Command deploy() {
        return sequence(
                        zero().unless(() -> zeroedPosition.isPresent()),
                        run(() -> io.setBrakeServoAngle(brakeDisengagedAngle.get()))
                                .withTimeout(0.5),
                        run(() -> {
                                    var position = releasePosition.get().plus(zeroedPosition.get());
                                    io.setPosition(position, NewtonMeters.zero());
                                    io.setBrakeServoAngle(brakeDisengagedAngle.get());
                                })
                                .until(() -> inputs.position.in(Radians)
                                        < releaseStartPosition.get().in(Radians)),
                        run(() -> {
                                    var position = releasePosition.get().plus(zeroedPosition.get());
                                    io.setPosition(position, NewtonMeters.zero());
                                    io.setReleaseServoSpeed(1.0);
                                    io.setBrakeServoAngle(brakeDisengagedAngle.get());
                                })
                                .withTimeout(3.0),
                        run(() -> {
                            var position = deployedPosition.get().plus(zeroedPosition.get());
                            io.setPosition(position, NewtonMeters.zero());
                            io.setReleaseServoSpeed(1.0);
                            io.setBrakeServoAngle(brakeDisengagedAngle.get());
                        }))
                .finallyDo(() -> io.stopReleaseServo())
                .unless(() -> DriverStation.getMatchTime() > 20)
                .withName("ClimbDeploy");
    }

    public Command retract() {
        return sequence(
                        zero().unless(() -> zeroedPosition.isPresent()),
                        run(() -> {
                                    io.setBrakeServoAngle(brakeEngagedAngle.get());
                                    io.setVoltage(retractVoltage.get());
                                })
                                .until(() -> inputs.position.in(Radians)
                                        > zeroedPosition.get().in(Radians)
                                                + climbedPosition.get().in(Radians)),
                        runOnce(() -> io.stop()))
                .withName("ClimbClimb");
    }

    public Command zero() {
        var debouncer = new Debouncer(0.1, DebounceType.kRising);
        return sequence(
                        runOnce(() -> debouncer.calculate(false)),
                        run(() -> {
                                    io.setTorqueCurrent(zeroTorqueCurrent.get());
                                    io.setBrakeServoAngle(brakeDisengagedAngle.get());
                                })
                                .until(() -> debouncer.calculate(inputs.velocity.lt(zeroVelocityLimit.get()))),
                        runOnce(() -> {
                            zeroedPosition = Optional.of(inputs.position);
                        }),
                        run(() -> {
                                    io.setPosition(
                                            preDeployedPosition.get().plus(zeroedPosition.get()), NewtonMeters.zero());
                                })
                                .until(() -> MathUtil.isNear(
                                        inputs.position.in(Radians),
                                        zeroedPosition.get().in(Radians)
                                                + preDeployedPosition.get().in(Radians),
                                        1.0)))
                .withName("ClimbZero");
    }

    public Command stop() {
        return sequence(run(() -> io.stop()).until(() -> inputs.velocity.gt(minBrakeSpeed.get())), run(() -> {
                    io.stop();
                }))
                .withName("ClimbStop");
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        io.setNeutralMode(neutralMode);
    }
}
