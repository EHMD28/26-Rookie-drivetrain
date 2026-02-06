package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeftModule = new SwerveModule(DriveConstants.frontLeftDrivingId,
            DriveConstants.frontLeftTurningId);
    private final SwerveModule frontRightModule = new SwerveModule(DriveConstants.frontRightDrivingId,
            DriveConstants.frontRightTurningId);
    private final SwerveModule backRightModule = new SwerveModule(DriveConstants.backRightDrivingId,
            DriveConstants.backRightTurningId);
    private final SwerveModule backLeftModule = new SwerveModule(DriveConstants.backLeftDrivingId,
            DriveConstants.backRightTurningId);
    private final SwerveModule[] allModules = {
            frontLeftModule,
            frontRightModule,
            backRightModule,
            backLeftModule
    };

    // TODO: Definitely going to need a gyroscope for keeping track of heading.

    /**
     * Drive the robot in the specified direction. Currently, the robot can either
     * translate or rotate, but not both. The "amount" for each parameter is a value
     * between -1.0 and 1.0.
     * 
     * @param x The horizontal translation (-1.0 is left, 1.0 is right).
     * @param y The vertical translation (-1.0 is back, 1.0 is forward).
     * @param rotation The amount to rotate (-1.0 is clockwise, 1.0 is counterclockwise)
     */
    public void drive(double x, double y, double rotation) {
        // You can check for 0 directly because the controller applies a deadband.
        boolean isTranslating = (x != 0.0) || (y != 0.0);
        boolean isTurning = rotation != 0.0;
        if (isTranslating && isTurning) {
            // TODO: Handle both.
            return;
        }

        if (isTranslating) {
            double angle = Math.atan2(y, x);
            turnAllTo(angle);
        } else {
            goToTurnPositions();
        }
    }

    /**
     * Turns all of the motors to a specific degree measure.
     * 
     * @param angleDeg Angle in degrees.
     */
    public void turnAllTo(double angleDeg) {
        for (SwerveModule module : allModules) {
            module.turnToAngle(angleDeg);
        }
    }

    public void goToTurnPositions() {
        // Roughly what the modules should look like
        // ┌---------┐ 
        // | /     \ |
        // |         |
        // | \     / |
        // └---------┘
        frontLeftModule.turnToAngle(45);
        frontRightModule.turnToAngle(315);
        backRightModule.turnToAngle(225);
        backLeftModule.turnToAngle(135);
    }

    public void driveAll(double speed) {
        for (SwerveModule module : allModules) {
            module.drive(speed);
        }
    }

    public void stopDriveAll() {
        for (SwerveModule module : allModules) {
            module.stopDrive();
        }
    }
}
