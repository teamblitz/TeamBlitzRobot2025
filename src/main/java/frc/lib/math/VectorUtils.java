package frc.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

public final class VectorUtils {
    public static <R extends Num> Vector<R> clamp(Vector<R> vec, double maxMagnitude) {
        if (vec.norm() == 0) return vec;

        return vec.unit().times(Math.min(vec.norm(), maxMagnitude));
    }

    public static <R extends Num> Vector<R> applyDeadband(
            Vector<R> vec, double deadband, double maxMagnitude) {
        if (vec.norm() == 0) return vec;

        return vec.unit().times(MathUtil.applyDeadband(vec.norm(), deadband, maxMagnitude));
    }

    public static <R extends Num> Vector<R> applyDeadband(Vector<R> vec, double deadband) {
        return applyDeadband(vec, deadband, 1.0);
    }

    private VectorUtils() {}
}
