package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.math.VectorUtils;
import frc.robot.OIConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final Timer maintainHeadingTimer = new Timer();
    private final double maintainHeadingDelay = .5;

    public Command joystickDrive(CommandSwerveDrivetrain drive,
                                 DoubleSupplier percentX,
                                 DoubleSupplier percentY,
                                 DoubleSupplier percentRotation,
                                 DoubleSupplier maxVelocity,
                                 DoubleSupplier maxAcceleration,
                                 DoubleSupplier maxAngularVelocity,
                                 boolean automaticallyCorrectHeading) {
        return Commands.runOnce(
                maintainHeadingTimer::reset
        ).andThen(
                drive.applyRequest(
                () -> {
                    var velocity = velocityFromJoysticks(percentX.getAsDouble(), percentY.getAsDouble(), maxVelocity.getAsDouble());

                    var omega = angularVelocityFromJoysticks(percentRotation.getAsDouble(), maxAngularVelocity.getAsDouble());


//                    if (automaticallyCorrectHeading && omega == 0 && maintainHeadingTimer.hasElapsed(maintainHeadingDelay)) {
//
//                        return fieldCentricFacingAngle
//                                .withVelocityX(velocity.get(0))
//                                .withVelocityY(velocity.get(0))
//                    }

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

        // apply input curve
        translationControl = translationControl.unit().times(OIConstants.Drive.TRANSLATION_INPUT_CURVE.apply(translationControl.norm()));

        // multiply by max velocity
        return translationControl.times(maxVelocity);
    }

    private double angularVelocityFromJoysticks(double percentRotation, double maxOmega) {
        // clamp rotation input
        var rotationControl = MathUtil.clamp(percentRotation, -1.0, 1.0);

        // apply curve
        rotationControl = OIConstants.Drive.SPIN_CURVE.apply(rotationControl);

        // multiply by max omega
        return rotationControl * maxOmega;
    }
}
