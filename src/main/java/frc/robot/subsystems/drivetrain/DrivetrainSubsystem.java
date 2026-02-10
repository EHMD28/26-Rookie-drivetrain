package frc.robot.subsystems.drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * The robot uses four swerve modules in a square configuration. The code is based on the template
 * provided by Rev Robotics at https://github.com/REVrobotics/MAXSwerve-Java-Template/tree/main.
 */
public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.frontLeftDrivingId,
            DriveConstants.frontLeftTurningId,
            DriveConstants.frontLeftAngularOffset);
    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.frontRightDrivingId,
            DriveConstants.frontRightTurningId,
            DriveConstants.frontRightAngularOffset);
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.backRightDrivingId,
            DriveConstants.backRightTurningId,
            DriveConstants.backRightAngularOffset);
    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.backLeftDrivingId,
            DriveConstants.backLeftTurningId,
            DriveConstants.backLeftAngularOffset);

    // TODO: Find right gyro model. 
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.maxSpeed;
        double ySpeedDelivered = ySpeed * DriveConstants.maxSpeed;
        double rotDelivered = rot * DriveConstants.maxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(gyro.getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }
}
