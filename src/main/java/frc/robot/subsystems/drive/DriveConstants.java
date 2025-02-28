package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
    public static final SwerveDrivetrainConstants drivetrainConstants = TunerConstants.DrivetrainConstants;
    public static final SwerveModuleConstants<?, ?, ?>[] moduleConstants = new SwerveModuleConstants[] {
        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight,
    };

    public static final Translation2d[] moduleTranslations = java.util.Arrays.stream(moduleConstants)
            .map(module -> new Translation2d(module.LocationX, module.LocationY))
            .toArray(Translation2d[]::new);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    public static final Matrix<N3, N1> odometryStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));

    public static final LinearVelocity maxSpeed = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration maxAcceleration = maxSpeed.div(Seconds.of(0.33));
    public static final AngularVelocity maxAngularRate = RotationsPerSecond.of(0.75);
    public static final AngularAcceleration maxAngularAcceleration = maxAngularRate.div(Seconds.of(0.33));

    public static final LinearVelocity maxSpeedExtended = MetersPerSecond.of(0.5);
    public static final LinearAcceleration maxAccelerationExtended = maxSpeedExtended.div(Seconds.of(0.33));

    public static final LinearVelocity maxSpeedFineControl = MetersPerSecond.of(1.0);
    public static final LinearAcceleration maxAccelerationFineControl = maxSpeedExtended.div(Seconds.of(0.33));

    public static final double translationDeadband = 0.1;
    public static final double rotationDeadband = 0.1;

    public static final double translationP = 10.0;
    public static final double translationI = 0.0;
    public static final double translationD = 0.0;

    public static final double rotationP = 7.0;
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.0;
}
