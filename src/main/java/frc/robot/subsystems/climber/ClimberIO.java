package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    /** All units are meters and radians */
    @AutoLog
    public class ClimberIOInputs {
        public double positionLeft;
        public double positionRight;

        public double velocityLeft;
        public double velocityRight;

        public double voltsLeft;
        public double voltsRight;

        public double torqueCurrentLeft;
        public double torqueCurrentRight;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setSpeedLeft(double percent) {}

    public default void setSpeedRight(double percent) {}

    public default void setMotionMagicLeft(double extension) {}

    public default void setMotionMagicRight(double extension) {}
}
