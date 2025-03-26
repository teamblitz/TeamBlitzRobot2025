package frc.lib.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class NanUtil {
    public static final TrapezoidProfile.State TRAPEZOID_NAN_STATE = new TrapezoidProfile.State(Double.NaN, Double.NaN);

    private NanUtil() {}
}
