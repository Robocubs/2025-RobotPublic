package frc.robot.subsystems.climb;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableMeasure;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.climb.ClimbConstants.*;

public class Climb extends SubsystemBase {
    private static final LoggedTunableMeasure<AngleUnit, Angle> climbedPosition =
            new LoggedTunableMeasure<>("Climb/ClimbedPosition", Radians.of(-3));
    private static final LoggedTunableMeasure<AngleUnit, Angle> deployedPosition =
            new LoggedTunableMeasure<>("Climb/DeployedPosition", Radians.of(-20));
    private static final LoggedTunableMeasure<AngleUnit, Angle> brakeEngagedAngle =
            new LoggedTunableMeasure<>("Climb/BrakeEngagedAngle", Degrees.of(90.0));
    private static final LoggedTunableMeasure<AngleUnit, Angle> brakeDisengagedAngle =
            new LoggedTunableMeasure<>("Climb/BrakeDisengagedAngle", Degrees.of(130));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> minBrakeSpeed =
            new LoggedTunableMeasure<>("Climb/ClimbedPosition", RadiansPerSecond.of(-0.2));
    private static final LoggedTunableBoolean zeroWithCurrent =
            new LoggedTunableBoolean("Climb/ZeroWithCurrent", false);
    private static final LoggedTunableMeasure<CurrentUnit, Current> zeroTorqueCurrent =
            new LoggedTunableMeasure<>("Climb/ZeroTorqueCurrent", Amps.of(5));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> zeroVelocityTarget =
            new LoggedTunableMeasure<>("Climb/ZeroVelocityTarget", RadiansPerSecond.of(10.0));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> zeroVelocityLimit =
            new LoggedTunableMeasure<>("Climb/ZeroVelocityLimit", RadiansPerSecond.of(0.1));
    private static final Angle positionTolerance = Radians.of(0.1);

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
                        run(() -> {
                                    io.setBrakeServoAngle(brakeDisengagedAngle.get());
                                    io.stop();
                                })
                                .withTimeout(Seconds.of(0.5)),
                        run(() -> {
                            var position = deployedPosition.get().plus(zeroedPosition.get());
                            io.setPosition(position, NewtonMeters.zero());
                            io.setBrakeServoAngle(brakeDisengagedAngle.get());

                            if (position.isNear(inputs.position, positionTolerance)
                                    && DriverStation.getMatchTime() < 20) {
                                io.setReleaseServoSpeed(1.0);
                            }
                        }))
                .finallyDo(() -> io.stopReleaseServo())
                .withName("ClimbDeploy");
    }

    public Command retract() {
        return sequence(
                        zero().unless(() -> zeroedPosition.isPresent()),
                        run(() -> {
                                    io.setBrakeServoAngle(brakeEngagedAngle.get());
                                    io.setVoltage(retractVoltage);
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
                                    if (zeroWithCurrent.get()) {
                                        io.setTorqueCurrent(zeroTorqueCurrent.get());
                                    } else {
                                        io.setVelocity(zeroVelocityTarget.get(), NewtonMeters.zero());
                                    }
                                })
                                .until(() -> debouncer.calculate(inputs.velocity.lt(zeroVelocityLimit.get()))),
                        runOnce(() -> {
                            zeroedPosition = Optional.of(inputs.position);
                            io.stop();
                        }))
                .withName("ClimbZero");
    }

    public Command stop() {
        return sequence(run(() -> io.stop()).until(() -> inputs.velocity.gt(minBrakeSpeed.get())), run(() -> {
                    io.setBrakeServoAngle(brakeEngagedAngle.get());
                    io.stop();
                }))
                .withName("ClimbStop");
    }
}
