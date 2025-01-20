package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorIOInputs {

        public double positionLeft;
        public double positionRight;

        public double velocityLeft;
        public double velocityRight;

        public double voltsLeft;
        public double voltsRight;

        public double torqueCurrentLeft; // do we need?
        public double torqueCurrentRight; // do we need?
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setSpeedLeft(double percent) {}

    public default void setSpeedRight(double percent) {}

    public default void setMotionMagicLeft(double extension) {}

    public default void setMotionMagicRight(double extension) {}
}
