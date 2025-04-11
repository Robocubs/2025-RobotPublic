package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithJoysticksEnhanced;
import frc.robot.subsystems.drive.controllers.PathController;
import frc.robot.util.GeometryUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class Drive extends SubsystemBase {
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.ApplyRobotSpeeds robotRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
            new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final RobotState robotState;
    private final PathController choreoTrajectoryController = new PathController();

    private @Getter Pose2d odometryPose = Pose2d.kZero;
    private @Getter double odometryPoseLastReset = 0.0;
    private @Getter SwerveModuleState[] moduleStates =
            Stream.generate(() -> new SwerveModuleState()).limit(4).toArray(SwerveModuleState[]::new);
    private @Getter SwerveModulePosition[] modulePositions =
            Stream.generate(() -> new SwerveModulePosition()).limit(4).toArray(SwerveModulePosition[]::new);

    public Drive(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        var measurements = Stream.of(inputs.inputs)
                .map(input -> new DriveMeasurement(
                        input.timestamp,
                        input.rawHeading,
                        input.modulePositions,
                        kinematics.toChassisSpeeds(input.moduleStates)))
                .toArray(DriveMeasurement[]::new);
        robotState.addDriveMeasurements(measurements);

        if (inputs.inputs.length > 0) {
            var lastInput = inputs.inputs[inputs.inputs.length - 1];
            odometryPose = lastInput.pose;
            moduleStates = lastInput.moduleStates;
            modulePositions = lastInput.modulePositions;
        }

        fieldCentricFacingAngle.HeadingController.setPID(7.0, 0.0, 0.0);
    }

    public void setRequest(SwerveRequest request) {
        io.setControl(request);
    }

    public void followPath(SwerveSample sample) {
        io.setControl(choreoTrajectoryController.update(sample, robotState.getPose(), robotState.getMaxSpeed()));
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        io.setNeutralMode(neutralMode);
    }

    public Command resetRotation() {
        return resetRotation(() -> RobotState.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg);
    }

    public Command resetRotation(Supplier<Rotation2d> rotation) {
        return Commands.runOnce(() -> {
                    var rotationValue = rotation.get();
                    io.resetRotation(rotationValue);
                    odometryPose = new Pose2d(odometryPose.getTranslation(), rotationValue);
                    odometryPoseLastReset = Logger.getTimestamp();
                })
                .withName("DriveResetRotation");
    }

    public Command withSpeeds(Supplier<ChassisSpeeds> chassisSpeeds) {
        return run(() -> io.setControl(robotRequest.withSpeeds(chassisSpeeds.get())))
                .withName("DriveWithSpeeds");
    }

    public Command withJoysticks(
            DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier fineControl) {
        return new DriveWithJoysticks(this, robotState, throttle, strafe, rotation, fineControl);
    }

    public Command withJoysticksEnhanced(
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier fineControl,
            BooleanSupplier leftCoralTrigger,
            BooleanSupplier rightCoralTrigger,
            BooleanSupplier algaeTrigger) {
        return new DriveWithJoysticksEnhanced(this, robotState, throttle, strafe, rotation, fineControl, algaeTrigger);
    }

    public DriveToPose toPose(Supplier<Pose2d> poseSupplier, boolean finishAtGoal, boolean slowMode) {
        return new DriveToPose(this, robotState, poseSupplier, finishAtGoal, slowMode);
    }

    public Command brake() {
        return run(() -> io.setControl(brakeRequest)).withName("DriveBrake");
    }

    public Command pointModulesAt(Supplier<Translation2d> target) {
        return run(() -> {
                    var pose = robotState.getPose();
                    var rotationToTarget =
                            target.get().minus(pose.getTranslation()).getAngle();
                    io.setControl(pointRequest.withModuleDirection(rotationToTarget.minus(pose.getRotation())));
                })
                .withName("DrivePointModules");
    }

    public Command pointModules(Supplier<Rotation2d> rotation) {
        return run(() -> io.setControl(pointRequest.withModuleDirection(rotation.get())))
                .withName("DrivePointModules");
    }

    public Command fineTuneClimb(DoubleSupplier throttle, DoubleSupplier strafe) {
        var maxSpeed = MetersPerSecond.of(2);
        var deadband = maxSpeed.times(0.1);
        return run(() -> {
            var currentRotation = robotState.getPose().getRotation();
            var rotationTargetRadians =
                    GeometryUtil.isNear(currentRotation, Rotation2d.kZero, Rotation2d.kCCW_90deg) ? 0 : Math.PI;
            var rotationTarget =
                    Rotation2d.fromRadians(odometryPose.getRotation().getRadians()
                            - robotState.getHeading().getRadians()
                            + rotationTargetRadians);

            var throttleValue = RobotState.isBlue() ? throttle.getAsDouble() : -throttle.getAsDouble();
            var strafeValue = RobotState.isBlue() ? strafe.getAsDouble() : -strafe.getAsDouble();

            setRequest(fieldCentricFacingAngle
                    .withVelocityX(maxSpeed.times(throttleValue))
                    .withVelocityY(maxSpeed.times(strafeValue))
                    .withDeadband(deadband)
                    .withTargetDirection(rotationTarget));
        });
    }
}
