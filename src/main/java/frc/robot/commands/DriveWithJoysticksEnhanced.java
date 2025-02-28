package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveWithJoysticksEnhanced extends Command {
    private static final double rotationTimeout = 0.25;
    private static final double rotationRateThreshold = Units.degreesToRadians(10);

    private static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> maxTranslationControllerOutput =
            new LoggedTunableMeasure<>("DriveWithJoysticks/MaxPidOutput", MetersPerSecond.of(1.0));
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
    private final BooleanSupplier coralTrigger;
    private final BooleanSupplier algaeTrigger;
    private final PIDController translationController =
            new PIDController(translationKP.get(), 0.0, 0, Constants.mainLoopPeriod.in(Seconds));

    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private double joystickLastTouched = 0.0;

    public DriveWithJoysticksEnhanced(
            Drive drive,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier fineControl,
            BooleanSupplier coralTrigger,
            BooleanSupplier algaeTrigger) {
        this.drive = drive;
        this.robotState = robotState;
        this.throttle = throttle;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fineControl = fineControl;
        this.coralTrigger = coralTrigger;
        this.algaeTrigger = algaeTrigger;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        headingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        translationController.setP(translationKP.get());
        fieldCentricFacingAngle.HeadingController.setP(headingKP.get());

        var fineControl = this.fineControl.getAsBoolean();
        var throttleValue = this.throttle.getAsDouble() * (RobotState.isBlue() ? 1.0 : -1.0);
        var strafeValue = this.strafe.getAsDouble() * (RobotState.isBlue() ? 1.0 : -1.0);
        var maxSpeedFromState = robotState.getMaxSpeed().in(MetersPerSecond);
        var maxSpeedFromControls = fineControl ? maxSpeedFineControl : maxSpeed;
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

        var rotationDemandIsZero = MathUtil.isNear(0.0, rotation, Constants.epsilon);
        if (!rotationDemandIsZero) {
            joystickLastTouched = Logger.getTimestamp();
        }

        var headingLocked = rotationDemandIsZero
                && (Logger.getTimestamp() > joystickLastTouched + rotationTimeout
                        || robotState.getRobotVelocity().omegaRadiansPerSecond < rotationRateThreshold);
        Logger.recordOutput("Commands/DriveWithJoysticksEnhanced/HeadingLocked", headingLocked);

        if (headingLocked) {
            var coralTrigger = this.coralTrigger.getAsBoolean();
            var algaeTrigger = this.algaeTrigger.getAsBoolean();

            Optional<Pose2d> targetPose = Optional.empty();
            // Target scoring coral
            if (coralTrigger && !robotState.isSelected(CoralMode.L1_CORAL) && robotState.isFacingReef()) {
                targetPose = robotState
                        .getClosestReefBranch()
                        .filter(tp -> tp.getTranslation().getDistance(pose.getTranslation()) < 2.0);
                headingSetpoint = targetPose.map(Pose2d::getRotation);
            }
            // Target algae pickup
            else if (algaeTrigger
                    && !robotState.hasGamePiece()
                    && robotState.inReefArea()
                    && robotState.isFacingReef()) {
                targetPose = robotState.getClosestReefAlgae();
                headingSetpoint = targetPose.map(Pose2d::getRotation);
            }
            // Target scoring processor
            else if (robotState.hasAlgae() && robotState.isFacingProcessor() && robotState.inProcessorArea()) {
                targetPose = Optional.of(FieldConstants.robotProcessorPose());
                headingSetpoint = targetPose.map(Pose2d::getRotation);
            }
            // Target barge
            else if (algaeTrigger && robotState.isFacingBarge() && robotState.inBargeArea()) {
                headingSetpoint = pose.getX() < FieldConstants.fieldCenter.getX()
                        ? Optional.of(Rotation2d.kZero)
                        : Optional.of(Rotation2d.k180deg);
            }
            // TODO: Rotate to coral station

            // Maintain heading
            if (headingSetpoint.isEmpty()) {
                headingSetpoint = Optional.of(pose.getRotation());
            }

            var rotationOffset =
                    Rotation2d.fromRadians(drive.getOdometryPose().getRotation().getRadians()
                            - pose.getRotation().getRadians());

            if (!fineControl && targetPose.isPresent()) {
                var translationToTarget = targetPose
                        .get()
                        .getTranslation()
                        .minus(pose.getTranslation())
                        .rotateBy(rotationOffset);
                var translationControllerSpeed = Math.min(
                        translationController.calculate(0.0, translationToTarget.getNorm()),
                        maxTranslationControllerOutput.get().in(MetersPerSecond));
                var translationControllerSpeeds =
                        new Translation2d(translationControllerSpeed, translationToTarget.getAngle());
                var outputAngle = new Rotation2d(
                        throttle + translationControllerSpeeds.getX(), strafe + translationControllerSpeeds.getY());
                var outputSpeed = inputLinearSpeed > 1.0 || translationToTarget.getNorm() < 0.01
                        ? inputLinearSpeed
                        : Math.min(translationControllerSpeed, 1.0);

                var translationSpeeds = new Translation2d(outputSpeed, outputAngle);
                throttle = translationSpeeds.getX();
                strafe = translationSpeeds.getY();
            }

            drive.setRequest(fieldCentricFacingAngle
                    .withVelocityX(throttle)
                    .withVelocityY(strafe)
                    .withTargetDirection(headingSetpoint.get().plus(rotationOffset)));

            Logger.recordOutput("Commands/DriveWithJoystick/HeadingSetpoint", headingSetpoint.get());
            Logger.recordOutput("Commands/DriveWithJoysticksEnhanced/TargetPose", targetPose.orElse(Pose2d.kZero));
        } else {
            headingSetpoint = Optional.empty();
            drive.setRequest(
                    fieldCentric.withVelocityX(throttle).withVelocityY(strafe).withRotationalRate(rotation));
        }
    }
}
