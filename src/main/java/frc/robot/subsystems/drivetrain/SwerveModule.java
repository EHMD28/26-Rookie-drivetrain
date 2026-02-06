package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Class for encapsulating the behavior of a single swerve module/
 */
public class SwerveModule {
    private final SparkMax drivingMotor;
    private final SparkMax turningMotor;
    // There should be an absolute encoder in the future. For now, it will suffice
    // to manually zero the wheels.
    private final RelativeEncoder turningEncoder;

    public SwerveModule(int drivingId, int turningId) {
        drivingMotor = new SparkMax(drivingId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningId, MotorType.kBrushless);
        turningEncoder = turningMotor.getEncoder();
    }

    /* Utility methods */

    public void drive(double speed) {
        drivingMotor.set(speed);
    }

    public void turn(double speed) {
        turningMotor.set(speed);
    }

    /**
     * Turn to a specific angle (in degrees). Position rotations are
     * counterclockwise, negative rotations are clockwise.
     * 
     * @param angleDeg
     */
    public void turnToAngle(double angleDeg) {

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
