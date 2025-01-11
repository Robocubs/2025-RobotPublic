package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveToPose extends Command {
    private static final double maxSpeedPercent = 0.8;
    private static final double maxAccelerationPercent = 0.8;

    private static final double translationTolerance = 0.05;
    private static final double rotationTolerance = Units.degreesToRadians(1.5);
    private static final double ffMinRadius = 0.2;
    private static final double ffMaxRadius = 0.8;
    private static final ChassisSpeeds zeroSpeeds = new ChassisSpeeds();

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final Drive drive;
    private final RobotState robotState;
    private final Supplier<Pose2d> targetPose;
    private final boolean finishAtGoal;
    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private Translation2d translationSetpoint = Translation2d.kZero;

    public DriveToPose(Drive drive, RobotState robotState, Supplier<Pose2d> targetPose, boolean finishAtGoal) {
        this.drive = drive;
        this.robotState = robotState;
        this.targetPose = targetPose;
        this.finishAtGoal = finishAtGoal;

        translationController = new ProfiledPIDController(
                translationP,
                translationI,
                translationD,
                new TrapezoidProfile.Constraints(maxSpeed * maxSpeedPercent, maxAcceleration * maxAccelerationPercent),
                0.02);
        rotationController = new ProfiledPIDController(
                rotationP,
                rotationI,
                rotationD,
                new TrapezoidProfile.Constraints(maxAngularRate, maxAngularAcceleration),
                0.02);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        var currentPose = robotState.getPose();
        var targetPose = this.targetPose.get();
        var rotationToTarget =
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        var fieldVelocity = robotState.getFieldVelocity();
        var velocityToTarget = ChassisSpeeds.fromFieldRelativeSpeeds(fieldVelocity, rotationToTarget).vxMetersPerSecond;
        var currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(currentDistance, -velocityToTarget);
        translationController.setTolerance(translationTolerance);
        rotationController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        rotationController.setTolerance(rotationTolerance);
        translationSetpoint = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        var currentPose = robotState.getPose();
        var targetPose = this.targetPose.get();

        var currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        var previousSetpointDistance = translationSetpoint.getDistance(targetPose.getTranslation());
        var ffScalar = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.5,
                1.0); // Fudge factor to prevent overdriving
        translationController.reset(
                Math.min(currentDistance, previousSetpointDistance), translationController.getSetpoint().velocity);
        var targetSpeed = translationController.calculate(currentDistance)
                + translationController.getSetpoint().velocity * ffScalar;
        if (translationController.atGoal()) {
            targetSpeed = 0.0;
        }

        var rotationTargetToRobot =
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        var translationalVelocity = new Translation2d(targetSpeed, 0).rotateBy(rotationTargetToRobot);
        translationSetpoint = new Pose2d(targetPose.getTranslation(), rotationTargetToRobot)
                .transformBy(new Transform2d(translationController.getSetpoint().position, 0.0, Rotation2d.kZero))
                .getTranslation();

        var rotationalVelocity = rotationController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians())
                + rotationController.getSetpoint().velocity;
        if (rotationController.atGoal()) {
            rotationalVelocity = 0.0;
        }

        drive.setRequest(applyRobotSpeeds.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationalVelocity.getX(),
                translationalVelocity.getY(),
                rotationalVelocity,
                currentPose.getRotation())));

        Logger.recordOutput(
                "Commands/DriveToPose/SetpointPose",
                new Pose2d(translationSetpoint, Rotation2d.fromRadians(rotationController.getSetpoint().position)));
        Logger.recordOutput("Commands/DriveToPose/TargetPose", targetPose);
        Logger.recordOutput(
                "Commands/DriveToPose/AtGoal", translationController.atGoal() && rotationController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRequest(applyRobotSpeeds.withSpeeds(zeroSpeeds));

        Logger.recordOutput("Commands/DriveToPose/SetpointPose", Pose2d.kZero);
        Logger.recordOutput("Commands/DriveToPose/TargetPose", Pose2d.kZero);
    }

    @Override
    public boolean isFinished() {
        return finishAtGoal && translationController.atGoal() && rotationController.atGoal();
    }
}
