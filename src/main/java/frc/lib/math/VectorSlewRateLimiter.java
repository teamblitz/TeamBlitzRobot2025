package frc.lib.math;

/**
 * A {@link edu.wpi.first.math.filter.SlewRateLimiter} for Nth dimensional space. Instead of limiting change for each of the axis,
 * this filter limits the magnitude of the desired change, with no impedance on its direction. This results in a filter whos
 * behavior is not dependent on coordinate system axis.
 */

import edu.wpi.first.math.*;

public class VectorSlewRateLimiter<R extends Num>  {
    private final double rateLimit;
    private Vector<R> prevVal;
    private Vector<R> lastDelta;
    private double prevTime;

    /**
     * Creates a new VectorSlewRateLimiter with the given rate limits and initial
     * value.
     *
     * @param rateLimit The rate-of-change limit, in units per second. This is expected to be positive.
     * @param initialValue The initial value of the input.
     */
    public VectorSlewRateLimiter(double rateLimit, Vector<R> initialValue) {
        this.rateLimit = Math.abs(rateLimit);
        this.prevVal = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();

        // Unsure how to create the proper sized vector with generics. So this just creates a zero vector that is the correct size.
        this.lastDelta = initialValue.times(0);
    }


    public Vector<R> calculate(Vector<R> input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;

        var valueDelta = input.minus(prevVal);
        if (valueDelta.norm() == 0) {
            lastDelta = valueDelta;
            return prevVal;
        }

        // Clamp the change to obey the max rate of change
        var clampedDelta = VectorUtils.clamp(valueDelta, rateLimit * elapsedTime);

        lastDelta = clampedDelta;

        prevVal = prevVal.plus(clampedDelta);
        prevTime = currentTime;

        return prevVal;
    }

    // The last calculated value
    public Vector<R> lastValue() {
        return prevVal;
    }

    // The last change (ie acceleration)
    public Vector<R> lastDelta() {
        return lastDelta;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(Vector<R> value) {
        prevVal = value;
        prevTime = MathSharedStore.getTimestamp();
    }

}
