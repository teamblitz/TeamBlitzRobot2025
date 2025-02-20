package frc.robot.subsystems.superstructure.elevator;

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

        public boolean topLimitSwitch;
        public boolean bottomLimitSwitch;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setSpeed(double percent) {}

    public default void setVolts(double volts) {}

    public default void setSetpoint(double position, double velocity, double nextVelocity) {}

    @Deprecated
    public default void setMotionMagicLeft(double extension) {}

    @Deprecated
    public default void setMotionMagicRight(double extension) {}

    public default void setPid(double p, double i, double d) {}

    public default void setFF(double kS, double kG, double kV, double kA) {}

    public default void setBrakeMode(boolean brakeMode) {}
}
