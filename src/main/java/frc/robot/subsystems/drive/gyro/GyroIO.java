package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public class GyroIOInputs {
        public double yaw; // counterclockwise positive
        public double yawRate;
        public double pitch;
        public double pitchRate;
        public double roll;
        public double rollRate;
        public boolean connected;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(GyroIOInputs inputs) {}
}
