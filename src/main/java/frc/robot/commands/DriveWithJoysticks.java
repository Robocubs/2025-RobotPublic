package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveWithJoysticks extends Command {
    private static final double rotationTimeout = 0.25;
    private static final double rotationRateThreshold = Units.degreesToRadians(10);

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * translationDeadband)
            .withRotationalDeadband(maxAngularRate * rotationDeadband)
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(maxSpeed * translationDeadband)
                    .withDriveRequestType(DriveRequestType.Velocity);

    private final Drive drive;
    private final RobotState robotState;
    private final DoubleSupplier throttle;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;

    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private double joystickLastTouched = 0.0;
    private double odometryPoseLastReset = 0.0;

    public DriveWithJoysticks(
            Drive drive,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation) {
        this.drive = drive;
        this.robotState = robotState;
        this.throttle = throttle;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        headingSetpoint = Optional.empty();
        fieldCentricFacingAngle.HeadingController.setPID(7.0, 0.0, 0.0);
    }

    @Override
    public void execute() {
        var throttle = this.throttle.getAsDouble() * maxSpeed * (RobotState.isBlue() ? 1.0 : -1.0);
        var strafe = this.strafe.getAsDouble() * maxSpeed * (RobotState.isBlue() ? 1.0 : -1.0);
        var rotation = this.rotation.getAsDouble() * maxAngularRate;

        var rotationDemandIsZero = MathUtil.isNear(0.0, rotation, fieldCentric.RotationalDeadband);
        if (!rotationDemandIsZero) {
            joystickLastTouched = Logger.getTimestamp();
        }

        var headingLocked = rotationDemandIsZero
                && (Logger.getTimestamp() > joystickLastTouched + rotationTimeout
                        || robotState.getRobotVelocity().omegaRadiansPerSecond < rotationRateThreshold);
        Logger.recordOutput("Commands/DriveWithJoystick/HeadingLocked", headingLocked);

        if (headingLocked) {
            if (odometryPoseLastReset != drive.getOdometryPoseLastReset()) {
                headingSetpoint = Optional.of(drive.getOdometryPose().getRotation());
                odometryPoseLastReset = drive.getOdometryPoseLastReset();
            } else if (headingSetpoint.isEmpty()) {
                headingSetpoint = Optional.of(drive.getOdometryPose().getRotation());
            }

            drive.setRequest(fieldCentricFacingAngle
                    .withVelocityX(throttle)
                    .withVelocityY(strafe)
                    .withTargetDirection(headingSetpoint.get()));

            Logger.recordOutput("Commands/DriveWithJoystick/HeadingSetpoint", headingSetpoint.get());
        } else {
            headingSetpoint = Optional.empty();
            drive.setRequest(
                    fieldCentric.withVelocityX(throttle).withVelocityY(strafe).withRotationalRate(rotation));
        }
    }
}
