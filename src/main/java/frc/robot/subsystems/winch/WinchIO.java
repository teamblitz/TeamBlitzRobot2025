package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
    void setPid(double p, double i, double d);

    @AutoLog
    public class WinchInputs {
        public double velocity;
        public double position;
        public double current;
    }

    default void updateInputs(WinchIO.WinchInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setMaxOutput(double maxOut) {};

    default void setPosition(double position) {}

    default void setMotionProfile(double position) {}
}
