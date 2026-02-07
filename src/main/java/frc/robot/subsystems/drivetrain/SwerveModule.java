package frc.robot.subsystems.drivetrain;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;

/**
 * Class for encapsulating the behavior of a single swerve module/
 */
public class SwerveModule {
    private final SparkMax drivingMotor;
    private final SparkClosedLoopController drivingController;
    // private final SparkClosedLoopController drivingController;
    private final SparkMax turningMotor;
    // There should be an absolute encoder in the future. For now, it will suffice
    // to manually zero the wheels.
    private final RelativeEncoder turningEncoder;
    private final PIDController turningController = new PIDController(
            0.1, // p
            0, // i
            0 // d
    );

    public SwerveModule(int drivingId, int turningId) {
        drivingMotor = new SparkMax(drivingId, MotorType.kBrushless);
        drivingController = drivingMotor.getClosedLoopController();
        turningMotor = new SparkMax(turningId, MotorType.kBrushless);
        turningEncoder = turningMotor.getEncoder();
        configureDrivingMotor();
        configureTurningMotor();
    }

    private void configureDrivingMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(0.1).i(0.0).d(0.0);
        drivingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureTurningMotor() {
        turningController.enableContinuousInput(-Math.PI, Math.PI);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig();
        encoderConfig.positionConversionFactor(DriveConstants.turningPositionConversionFactor);
        motorConfig.apply(encoderConfig);
        turningMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningEncoder.setPosition(0);
    }

    /* Getters and Setters */

    public double getAngleRad() {
        return turningEncoder.getPosition();
    }

    /* Utility methods */

    /**
     * Drives the driving motor at a specific speed.
     * 
     * @param rpm How fast the spin the motor in RPM.
     */
    public void drive(double rpm) {
        turningController.setSetpoint(rpm);
    }

    /**
     * 
     * @param angleRad The target angle in radians. Should be from -π to π.
     */
    public void turnToAngle(double angleRad) {
        double target = turningController.calculate(getAngleRad(), angleRad);
        turningMotor.set(target);
    }

    public void stopDrive() {
        drivingMotor.stopMotor();
    }

    public void stopTurn() {
        turningMotor.stopMotor();
    }

    public void stopBoth() {
        stopDrive();
        stopTurn();
    }
}
