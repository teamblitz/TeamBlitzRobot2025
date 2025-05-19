package frc.lib.math.controller;

import edu.wpi.first.math.MathUtil;

public class DualPhaseProfile {

    /**
     * Computes a smooth velocity toward the origin using a two-phase control law:
     * <ul>
     *   <li>For positions far from the origin, applies a deceleration profile based on a constant max acceleration {@code a}.</li>
     *   <li>For positions near the origin, switches to proportional control using gain {@code p}.</li>
     * </ul>
     *
     * <p>The transition point is chosen to make the velocity and acceleration continuous.
     * This ensures smooth motion with no sudden changes in speed or force.</p>
     *
     * <p>The raw velocity is then clamped in magnitude to {@code maxV}.
     *
     * <p>Switching occurs at
     * <code>Z = a / (p*p)</code>.  For <code>x &gt; Z</code> we use
     * <code>v = -√[a*(2*p*p*x - a)]/p</code>, otherwise <code>v = -p * x</code>.
     *
     * <p>You can play around with this control law <a href="https://www.desmos.com/calculator/v9xvetkmzg">here</a></p>
     *
     * @param x     current position (distance from origin, x ≥ 0
     * @param kP     proportional gain for terminal phase (p > 0)
     * @param maxA     maximum deceleration magnitude (a > 0)
     * @param maxV  maximum allowed speed (maxV ≥ 0).  The computed v will be clamped to [−maxV, +maxV].
     * @return the commanded velocity, guaranteed to satisfy |v| ≤ maxV
     * */
    public static double ComputeApproachVelocity(
            double x,
            double kP,
            double maxA,
            double maxV
    ) {
        if (maxA <= 0 || kP <= 0 || maxV < 0 || x < 0) {
            throw new IllegalArgumentException("Invalid arguments: x >= 0, maxA > 0, kP > 0, maxV >= 0 required");
        }
        double transitionPoint = maxA / (kP * kP);

        double vRaw;
        if (x > transitionPoint) {
            var a_over_p = maxA / kP;

            vRaw = -Math.sqrt(2 * maxA * x - (a_over_p * a_over_p));
        } else {
            vRaw = -kP * x;
        }
        // clamp to ±maxV
        return MathUtil.clamp(vRaw, -maxV, maxV);
    }
}
