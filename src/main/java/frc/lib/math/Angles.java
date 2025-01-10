package frc.lib.math;

public class Angles {
    private Angles() {}

    /**
     * @deprecated for removal, Use {@link edu.wpi.first.math.MathUtil#angleModulus(double)}
     *     instead.
     */
    @Deprecated(forRemoval = true)
    public static double wrapAnglePi(double degrees) {
        return wrapAngle(degrees, -Math.PI, Math.PI);
    }

    /**
     * @deprecated for removal, Use {@link edu.wpi.first.math.MathUtil#inputModulus(double, double,
     *     double)} instead.
     */
    @Deprecated(forRemoval = true)
    public static double wrapAngle(double radians, double min, double max) {
        double angle = radians % (2 * Math.PI);
        if (angle > max) {
            angle -= 2 * Math.PI;
        } else if (angle < min) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
