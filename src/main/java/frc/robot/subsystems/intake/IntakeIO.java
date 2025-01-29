package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class EndEffectorInputs {
        public double rpm;
        public double current;
    }

    default void updateInputs(EndEffectorInputs inputs) {}

    default void setSpeed(double speed) {}
}
