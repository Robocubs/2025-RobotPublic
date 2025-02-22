package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveWithJoysticksEnhanced extends Command {
    private static final double rotationTimeout = 0.25;
    private static final double rotationRateThreshold = Units.degreesToRadians(10);
    private static final double maxTranslationControllerOutput = 0.5;

    private static final LoggedTunableNumber translationKP =
            new LoggedTunableNumber("DriveWithJoysticks/TranslationKP", 4.0);
    private static final LoggedTunableNumber headingKP = new LoggedTunableNumber("DriveWithJoysticks/HeadingKP", 7.0);

    private final SwerveRequest.FieldCentric fieldCentric =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
            new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

    private final Drive drive;
    private final RobotState robotState;
    private final DoubleSupplier throttle;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;
    private final BooleanSupplier fineControl;
    private final PIDController translationController =
            new PIDController(translationKP.get(), 0.0, 0, Constants.mainLoopPeriod.in(Seconds));

    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private double joystickLastTouched = 0.0;
    private double odometryPoseLastReset = 0.0;

    public DriveWithJoysticksEnhanced(
            Drive drive,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier fineControl) {
        this.drive = drive;
        this.robotState = robotState;
        this.throttle = throttle;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fineControl = fineControl;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        headingSetpoint = Optional.empty();
        translationController.setPID(translationKP.get(), 0.0, 0.0);
        fieldCentricFacingAngle.HeadingController.setPID(headingKP.get(), 0.0, 0.0);
    }

    @Override
    public void execute() {
        var throttleValue = this.throttle.getAsDouble() * (RobotState.isBlue() ? 1.0 : -1.0);
        var strafeValue = this.strafe.getAsDouble() * (RobotState.isBlue() ? 1.0 : -1.0);
        var maxSpeedFromState = robotState.getMaxSpeed().in(MetersPerSecond);
        var maxSpeedFromControls = this.fineControl.getAsBoolean() ? maxSpeedFineControl : maxSpeed;
        var maxAllowedSpeed = Math.min(maxSpeedFromState, maxSpeedFromControls.in(MetersPerSecond));

        var inputLinearSpeed =
                MathUtil.applyDeadband(Math.hypot(throttleValue, strafeValue), translationDeadband) * maxAllowedSpeed;
        var inputTranslationSpeeds = inputLinearSpeed > 1e-6
                ? new Translation2d(inputLinearSpeed, new Rotation2d(throttleValue, strafeValue))
                : Translation2d.kZero;
        var throttle = inputTranslationSpeeds.getX();
        var strafe = inputTranslationSpeeds.getY();

        var rotation = MathUtil.applyDeadband(this.rotation.getAsDouble(), rotationDeadband)
                * maxAngularRate.in(RadiansPerSecond);
        var pose = robotState.getPose();

        var rotationDemandIsZero = MathUtil.isNear(0.0, rotation, fieldCentric.RotationalDeadband);
        if (!rotationDemandIsZero) {
            joystickLastTouched = Logger.getTimestamp();
        }

        var headingLocked = rotationDemandIsZero
                && (Logger.getTimestamp() > joystickLastTouched + rotationTimeout
                        || robotState.getRobotVelocity().omegaRadiansPerSecond < rotationRateThreshold);
        Logger.recordOutput("Commands/DriveWithJoystick/HeadingLocked", headingLocked);

        if (headingLocked) {
            Optional<Translation2d> targetTranslation = Optional.empty();
            // Target scoring coral
            if (robotState.hasLongCoral() && robotState.isFacingReef()) {
                var targetPose = robotState
                        .getClosestReefBranch()
                        .filter(tp -> tp.getTranslation().getDistance(pose.getTranslation()) < 0.5);
                if (targetPose.isPresent()) {
                    targetTranslation = Optional.of(targetPose.get().getTranslation());
                    headingSetpoint = Optional.of(targetPose.get().getRotation().plus(Rotation2d.k180deg));
                }
            }
            // Target algae pickup
            else if (!robotState.hasGamePiece() && robotState.inReefArea()) {
                var targetPose = robotState.getClosestReefAlgae();
                if (targetPose.isPresent()) {
                    targetTranslation = Optional.of(targetPose.get().getTranslation());
                    headingSetpoint = Optional.of(targetPose.get().getRotation().plus(Rotation2d.k180deg));
                }
            }
            // Target scoring processor
            else if (robotState.hasAlgae() && robotState.isFacingProcessor() && robotState.inProcessorArea()) {
                var targetPose = FieldConstants.robotProcessorPose();
                targetTranslation = Optional.of(targetPose.getTranslation());
                headingSetpoint = Optional.of(targetPose.getRotation());
            }
            // Target barge
            else if (robotState.hasAlgae() && robotState.isFacingBarge() && robotState.inBargeArea()) {
                headingSetpoint = pose.getX() < FieldConstants.fieldCenter.getX()
                        ? Optional.of(Rotation2d.kZero)
                        : Optional.of(Rotation2d.k180deg);
            }
            // Maintain heading after odometry reset
            else if (odometryPoseLastReset != drive.getOdometryPoseLastReset()) {
                headingSetpoint = Optional.of(drive.getOdometryPose().getRotation());
                odometryPoseLastReset = drive.getOdometryPoseLastReset();
            }
            // Maintain heading
            else if (headingSetpoint.isEmpty()) {
                headingSetpoint = Optional.of(drive.getOdometryPose().getRotation());
            }

            if (targetTranslation.isPresent()) {
                var translationToTarget = targetTranslation.get().minus(pose.getTranslation());
                var translationControllerSpeed = Math.min(
                        translationController.calculate(0.0, translationToTarget.getNorm()),
                        maxTranslationControllerOutput);
                var translationControllerSpeeds =
                        new Translation2d(translationControllerSpeed, translationToTarget.getAngle());
                var outputAngle = new Rotation2d(
                        throttle + translationControllerSpeeds.getX(), strafe + translationControllerSpeeds.getY());
                var translationSpeeds = new Translation2d(inputLinearSpeed, outputAngle);
                throttle = translationSpeeds.getX();
                strafe = translationSpeeds.getY();
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
