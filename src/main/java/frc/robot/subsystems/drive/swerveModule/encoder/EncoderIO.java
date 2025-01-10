package frc.robot.subsystems.drive.swerveModule.encoder;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
    @AutoLog
    public static class EncoderIOInputs {
        public double position;
    }

    public default void updateInputs(EncoderIOInputs inputs) {}
}
