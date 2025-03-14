package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberInputs {
        public double rpm;
        public double position;
        public double current;
    }

    default void updateInputs(ClimberIO.ClimberInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setMotionProfile(double position) {}
}
