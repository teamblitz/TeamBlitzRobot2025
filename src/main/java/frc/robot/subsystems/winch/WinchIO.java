package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
    @AutoLog
    public class WinchInputs {
        public double rpm;
        public double position;
        public double current;
    }

    default void updateInputs(WinchIO.WinchInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setMotionProfile(double position) {}
}
