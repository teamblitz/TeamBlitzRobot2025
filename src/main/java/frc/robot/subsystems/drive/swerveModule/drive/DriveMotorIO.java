package frc.robot.subsystems.drive.swerveModule.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveMotorIO {
    @AutoLog
    public static class DriveMotorInputs {
        public double velocity;
        public double position;
        public double volts;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveMotorInputs inputs) {}

    public default void setDrivePercent(double percent) {}

    /**
     * Sets the velocity setpoint for the drive pid controller
     *
     * @param setpoint speed in meters per second.
     * @param ffVolts Feed forward in volts.
     */
    public default void setSetpoint(double setpoint, double ffVolts) {}

    /** Configure the PID constants */
    public default void configurePID(double p, double i, double d) {}

    public default void setBrakeMode(boolean enabled) {}
}
