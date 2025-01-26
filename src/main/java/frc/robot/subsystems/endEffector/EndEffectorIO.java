package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public class EndEffectorInputs {
        public double rpm;
        public double current;
    }

    default void updateInputs(EndEffectorInputs inputs) {}

    default void setSpeed(double speed) {}
}
