package frc.robot.subsystems.superstructure.wrist;

import frc.robot.subsystems.superstructure.wrist.Wrist;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public class WristIOInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double volts;
        public double torqueCurrent;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setSetpoint(double position, double velocity, double acceleration) {}

    public default void setPid(double p, double i, double d) {}
}

