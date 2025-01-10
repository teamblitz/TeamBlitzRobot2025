package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {
        public double rpm;
        public double current;

        public boolean breakBeam;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void set(double speed) {}
}
