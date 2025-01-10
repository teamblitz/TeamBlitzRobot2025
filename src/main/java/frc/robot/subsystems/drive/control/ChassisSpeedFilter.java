package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public abstract class ChassisSpeedFilter {
    protected final Drive drive;
    private final boolean functionFieldRelative;

    public ChassisSpeedFilter(Drive drive, boolean fieldRelative) {
        this.drive = drive;
        this.functionFieldRelative = fieldRelative;
    }

    protected abstract ChassisSpeeds apply(ChassisSpeeds chassisSpeeds);

    public void reset() {}

    public ChassisSpeeds filterSpeeds(ChassisSpeeds speeds, boolean inputFieldRelative) {
        // If the input and function are both either field-relative or both robot-relative
        if (functionFieldRelative == inputFieldRelative) {
            // No transformation needed, simply apply the function
            return apply(speeds);
        }

        // If the input is field-relative but the function expects robot-relative
        else if (inputFieldRelative) {
            // Convert the input from field-relative to robot-relative,
            // apply the function, then convert it back to field-relative
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                    apply(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getYaw())),
                    drive.getYaw());
        }

        // If the input is robot-relative but the function expects field-relative
        else {
            // Convert the input from robot-relative to field-relative,
            // apply the function, then convert it back to robot-relative
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    apply(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getYaw())),
                    drive.getYaw());
        }
    }
}
