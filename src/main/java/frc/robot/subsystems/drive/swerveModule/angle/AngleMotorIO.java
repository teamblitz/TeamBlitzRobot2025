package frc.robot.subsystems.drive.swerveModule.angle;

import org.littletonrobotics.junction.AutoLog;

public interface AngleMotorIO {
    @AutoLog
    public static class AngleMotorInputs {
        public double rotation;
        public double angularVelocity;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(AngleMotorInputs inputs) {}

    /**
     * Sets the position setpoint for the angle pid controller
     *
     * @param setpoint position in radians
     */
    public default void setSetpoint(double setpoint) {}

    /** Configure the PID constants */
    public default void configurePID(double p, double i, double d) {}

    public default void seedPosition(double position) {}
}
