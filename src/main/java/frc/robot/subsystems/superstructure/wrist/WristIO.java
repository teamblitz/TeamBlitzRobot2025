package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double volts;
        public double torqueCurrent;
        //public double feedforward;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setSetpoint(double position, double velocity, double acceleration) {}

    public default void setPid(double p, double i, double d) {}

    public default void setPercent(double percent) {}

    public default void setFF(double kS, double kV, double kA, double kG) {}

    public default void setBreakMode(boolean breakMode) {}
}
