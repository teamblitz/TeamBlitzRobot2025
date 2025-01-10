package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** All units are meters and radians */
    @AutoLog
    public class ArmIOInputs {
        public double rotation;
        public double rotationDeg;
        public double angularVelocity;
        public double absRotation;

        public double rawAbsEncoder;
        public double rawAbsEncoderDeg;

        public boolean topRotationLimit;
        public boolean bottomRotationLimit;

        public boolean encoderConnected;

        public double volts;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setRotationSetpoint(double degrees, double arbFFVolts) {}

    public default void setArmSpeed(double percent) {}

    public default void setArmVolts(double volts) {}

    public default void seedArmPosition(boolean assumeStarting) {}

    public default void checkLimitSwitches() {}

    public default void setBrake(boolean brake) {}

    public default void setPid(double kP, double kI, double kD) {}

    public default void stop() {}
}
