package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OIConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TeleopSwerve extends Command {
    private final Drive drive;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final DoubleSupplier headingSup;

    public TeleopSwerve(
            Drive s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            DoubleSupplier headingSup) {
        this.drive = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.headingSup = headingSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal =
                MathUtil.applyDeadband(
                        translationSup.getAsDouble(), OIConstants.Drive.STICK_DEADBAND);
        double strafeVal =
                MathUtil.applyDeadband(strafeSup.getAsDouble(), OIConstants.Drive.STICK_DEADBAND);
        double rotationVal =
                MathUtil.applyDeadband(rotationSup.getAsDouble(), OIConstants.Drive.STICK_DEADBAND);

        Logger.recordOutput("DriveCommand/translation", translationVal);
        Logger.recordOutput("DriveCommand/strafe", strafeVal);
        Logger.recordOutput("DriveCommand/rot", rotationVal);

        if (!DriverStation.isAutonomous()) {
            /* Drive */
            drive.angleDrive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Drive.MAX_SPEED),
                    rotationVal * Constants.Drive.MAX_ANGULAR_VELOCITY,
                    headingSup.getAsDouble(),
                    !robotCentricSup.getAsBoolean(),
                    true,
                    true,
                    !Double.isNaN(headingSup.getAsDouble()));
        }
    }
}
