package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberInputs {
        public double velocity;
        public double position;
        public double current;

        public double absolutePosition;
    }

    default void updateInputs(ClimberIO.ClimberInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setVolts(double volts) {}

    default void setMotionProfile(double position) {}

    default void setBrakeMode(boolean brakeMode) {}
}
