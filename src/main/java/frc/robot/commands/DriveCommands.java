package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.math.VectorUtils;
import frc.lib.util.Capture;
import frc.robot.OIConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)// Use open-loop control for drive motors
            .withHeadingPID(10, 0, 0);


    private final Timer maintainHeadingTimer = new Timer();
    private final double maintainHeadingDelay = .5;

    private final CommandSwerveDrivetrain drive;

    public DriveCommands(
            CommandSwerveDrivetrain drive
    ) {
        this.drive = drive;
    }


    public Command joystickDrive(DoubleSupplier percentX,
                                 DoubleSupplier percentY,
                                 DoubleSupplier percentRotation,
                                 DoubleSupplier maxVelocity,
                                 DoubleSupplier maxAcceleration,
                                 DoubleSupplier maxAngularVelocity,
                                 boolean automaticallyCorrectHeading) {

        Capture<Rotation2d> headingSetpoint = new Capture<>(null);

        return Commands.runOnce(
                () -> {
                    headingSetpoint.inner = drive.getHeading();
                    maintainHeadingTimer.restart();
                }
        ).andThen(
                drive.applyRequest(
                () -> {
                    var velocity = velocityFromJoysticks(percentX.getAsDouble(), percentY.getAsDouble(), maxVelocity.getAsDouble());

                    var omega = angularVelocityFromJoysticks(percentRotation.getAsDouble(), maxAngularVelocity.getAsDouble());

                    Logger.recordOutput("drive/joystick/speeds", new ChassisSpeeds(
                            velocity.get(0),
                            velocity.get(1),
                            omega
                    ));

                    if (automaticallyCorrectHeading) {
                        if (omega != 0) { // Maintain heading invalid, reset timer and set setpoint to null
                            maintainHeadingTimer.reset();
                            maintainHeadingTimer.stop();
                        } else {
                            maintainHeadingTimer.start();
                        }

                        boolean maintainHeadingValid = maintainHeadingTimer.hasElapsed(maintainHeadingDelay);

                        Logger.recordOutput("drive/joystick/maintainHeading/timer", maintainHeadingTimer.get());
                        Logger.recordOutput("drive/joystick/maintainHeading/valid", maintainHeadingValid);

                        if (maintainHeadingValid) {
                            if (headingSetpoint.get() == null) { // Set the setpoint if not yet set
                                headingSetpoint.inner = drive.getHeading();
                            }

                            Logger.recordOutput("drive/joystick/maintainHeading/setpoint", headingSetpoint.get());

                            return fieldCentricFacingAngle
                                    .withVelocityX(velocity.get(0))
                                    .withVelocityY(velocity.get(1))
                                    .withTargetDirection(headingSetpoint.get());
                        } else {
                            headingSetpoint.inner = null;

                            // Logging NaN is eh, but 0 isn't really appropriate here.
                            // Generally best if we want this behavior to NOT store NaN intentionally to avoid NaN poisoning.
                            Logger.recordOutput("drive/joystick/maintainHeading/setpoint", Rotation2d.fromRotations(Double.NaN));
                        }
                    }

                    return fieldCentric
                            .withVelocityX(velocity.get(0))
                            .withVelocityY(velocity.get(1))
                            .withRotationalRate(omega);
                }
        ));
    }


    private Vector<N2> velocityFromJoysticks(double percentX, double percentY, double maxVelocity) {
        // Express control effort as a vector
        var translationControl = VecBuilder.fill(percentX, percentY);

        // Clamp to the range 0:1
        translationControl = VectorUtils.clamp(translationControl, 1.0);

        // apply deadband
        translationControl = VectorUtils.applyDeadband(translationControl, OIConstants.Drive.TRANSLATION_DEADBAND);

        // Filter out zero control to avoid NaN poisoning.
        if (translationControl.norm() == 0) {
            return translationControl;
        }

        // apply input curve
        translationControl = translationControl.unit().times(OIConstants.Drive.TRANSLATION_INPUT_CURVE.apply(translationControl.norm()));

        // multiply by max velocity
        return translationControl.times(maxVelocity);
    }

    private double angularVelocityFromJoysticks(double percentRotation, double maxOmega) {
        // clamp rotation input
        var rotationControl = MathUtil.clamp(percentRotation, -1.0, 1.0);

        rotationControl = MathUtil.applyDeadband(percentRotation, OIConstants.Drive.ROTATION_DEADBAND);

        // apply curve
        rotationControl = OIConstants.Drive.SPIN_CURVE.apply(rotationControl);

        // multiply by max omega
        return rotationControl * maxOmega;
    }
}
