package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeInputs {
        public double rpm;
        public double current;

        public boolean breakBeam;
    }

    default void updateInputs(IntakeInputs inputs) {}

    default void setSpeed(double speed) {}
}
