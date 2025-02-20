package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double volts;
        public double torqueCurrent;

        public double absoluteEncoderPosition;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setSetpoint(double position, double velocity, double nextVelocity) {}

    public default void setPid(double p, double i, double d) {}

    public default void setPercent(double percent) {}

    public default void setFF(double kS, double kG, double kV, double kA) {}

    public default void setBrakeMode(boolean brakeMode) {}
}
