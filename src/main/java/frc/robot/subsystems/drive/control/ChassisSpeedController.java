package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public interface ChassisSpeedController extends Supplier<ChassisSpeeds> {
    default boolean isFieldRelative() {
        return false;
    }

    default boolean isOpenLoop() {
        return false;
    }

    static ChassisSpeedController from(
            Supplier<ChassisSpeeds> speeds,
            Supplier<Boolean> fieldRelative,
            Supplier<Boolean> openLoop) {
        return new ChassisSpeedController() {
            @Override
            public ChassisSpeeds get() {
                return speeds.get();
            }

            @Override
            public boolean isFieldRelative() {
                return fieldRelative.get();
            }

            @Override
            public boolean isOpenLoop() {
                return openLoop.get();
            }
        };
    }
}
