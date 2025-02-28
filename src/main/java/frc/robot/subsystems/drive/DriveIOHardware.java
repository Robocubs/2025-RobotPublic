package frc.robot.subsystems.drive;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.TunerConstants.TunerSwerveDrivetrain;

public class DriveIOHardware implements DriveIO {
    protected final TunerSwerveDrivetrain swerveDrivetrain;
    private final Queue<SwerveDriveInputs> inputsQueue = new ArrayBlockingQueue<>(20);
    private final Lock inputsLock = new ReentrantLock();

    public DriveIOHardware() {
        swerveDrivetrain =
                new TunerSwerveDrivetrain(DriveConstants.drivetrainConstants, DriveConstants.moduleConstants);
        swerveDrivetrain.registerTelemetry(this::addTelemetry);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputsLock.lock();
        try {
            inputs.inputs = inputsQueue.toArray(new SwerveDriveInputs[inputsQueue.size()]);
            inputsQueue.clear();
        } finally {
            inputsLock.unlock();
        }

        inputs.rotation = swerveDrivetrain.getRotation3d();
    }

    private void addTelemetry(SwerveDriveState state) {
        var inputs = SwerveDriveInputs.fromSwerveDriveState(state);
        inputsLock.lock();
        try {
            inputsQueue.add(inputs);
        } finally {
            inputsLock.unlock();
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        swerveDrivetrain.setControl(request);
        swerveDrivetrain.getOperatorForwardDirection();
    }

    @Override
    public void resetRotation(Rotation2d rotation2d) {
        swerveDrivetrain.resetRotation(rotation2d);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        swerveDrivetrain.configNeutralMode(neutralMode);
    }
}
