package frc.robot.subsystems.drive.swerveModule.drive;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;

public class DriveMotorIOSim implements DriveMotorIO {

    double position;
    double velocity;

    boolean needsUpdate;

    public DriveMotorIOSim() {
        RobotModeTriggers.disabled()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            velocity = 0;
                                            needsUpdate = true;
                                        })
                                .ignoringDisable(true));
    }

    @Override
    public void updateInputs(DriveMotorIO.DriveMotorInputs inputs) {
        if (needsUpdate) {
            inputs.position = (position += velocity * Constants.LOOP_PERIOD_SEC);
            inputs.velocity = velocity;
            needsUpdate = false;
        }
        inputs.volts =
                0; // Volts also isn't simulated. No point as we aren't characterizing our sim
    }

    @Override
    public void setDrivePercent(double percent) {
        velocity = percent * Constants.Drive.MAX_SPEED;
        needsUpdate = true;
        // TODO: For the sake of time this is not simulated.
        // (it was stupid easy to implement)
    }

    @Override
    public void setSetpoint(double setpoint, double ffVolts) {
        velocity = setpoint;
        needsUpdate = true;
    }
}
