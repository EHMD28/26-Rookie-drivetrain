package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Configs.SwerveConfig;

/**
 * Class for encapsulating the behavior of a single swerve module/
 */
public class SwerveModule {
    private final SparkMax drivingMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;
    // Initializes the swerve module with a speed of zero meters per second.
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingId, int turningId, double angularOffset) {
        /* Driving motor. */
        drivingMotor = new SparkMax(drivingId, MotorType.kBrushless);
        drivingEncoder = drivingMotor.getEncoder();
        drivingClosedLoopController = drivingMotor.getClosedLoopController();
        drivingMotor.configure(SwerveConfig.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        /* Turning motor. */
        turningMotor = new SparkMax(turningId, MotorType.kBrushless);
        turningEncoder = turningMotor.getAbsoluteEncoder();
        turningClosedLoopController = turningMotor.getClosedLoopController();
        turningMotor.configure(SwerveConfig.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        /* Other */
        chassisAngularOffset = angularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(drivingEncoder.getVelocity(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }
}
