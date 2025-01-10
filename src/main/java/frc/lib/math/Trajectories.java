package frc.lib.math;

public final class Trajectories {
    private Trajectories() {}
    ;

    /**
     * <a (x, y)
     * href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Angle
     * θ required to hit coordinate (x, y)</a> To hit a target at range x and altitude y when fired
     * from (0,0) and with initial speed v the required angle(s) of launch θ are:
     *
     * @param x delta x
     * @param y delta y
     * @param v initial velocity
     * @param g acceleration due to gravity
     * @return angle of release in radians
     */
    public static double angleRequiredToHitCoordinate(double x, double y, double v, double g) {
        return Math.atan(
                (v * v - Math.sqrt(Math.pow(v, 4) - g * (g * (x * x) + 2 * y * (v * v))))
                        / (g * x));
    }
}
