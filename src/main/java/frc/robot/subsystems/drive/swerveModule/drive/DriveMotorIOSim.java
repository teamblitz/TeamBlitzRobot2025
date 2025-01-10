package frc.robot.subsystems.drive.swerveModule.drive;

import frc.robot.Constants;

public class DriveMotorIOSim implements DriveMotorIO {

    double position;
    double velocity;

    @Override
    public void updateInputs(DriveMotorIO.DriveMotorInputs inputs) {
        inputs.position = (position += velocity * Constants.LOOP_PERIOD_SEC);
        inputs.velocity = velocity;
        inputs.volts =
                0; // Volts also isn't simulated. No point as we aren't characterizing our sim
    }

    @Override
    public void setDrivePercent(double percent) {
        // TODO: For the sake of time this is not simulated.
    }

    @Override
    public void setSetpoint(double setpoint, double ffVolts) {
        velocity = setpoint;
    }
}
