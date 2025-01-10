package frc.robot.subsystems.drive.swerveModule;

public class SwerveModuleConfiguration {
    public enum MotorType {
        KRAKEN,
        NEO
    }

    public enum EncoderType {
        HELIUM,
        CANCODER
    }

    public final MotorType drive;
    public final MotorType angle;
    public final EncoderType encoder;

    public SwerveModuleConfiguration(
            MotorType driveMotor, MotorType angleMotor, EncoderType encoder) {
        this.drive = driveMotor;
        this.angle = angleMotor;
        this.encoder = encoder;
    }
}
