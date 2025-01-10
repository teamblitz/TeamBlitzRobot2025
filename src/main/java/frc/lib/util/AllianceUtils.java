package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtils {
    private AllianceUtils() {}

    public static boolean isBlue() {
        return DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }
}
