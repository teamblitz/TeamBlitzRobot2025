package frc.lib.util;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class DriveUtil {
    public static SwerveSample sample(
            Translation2d trans, Rotation2d rot, double vx, double vy, double omega) {
        return new SwerveSample(
                0,
                trans.getX(),
                trans.getY(),
                rot.getRadians(),
                vx,
                vy,
                omega,
                0,
                0,
                0,
                new double[4],
                new double[4]);
    }

    private DriveUtil() {}
}
