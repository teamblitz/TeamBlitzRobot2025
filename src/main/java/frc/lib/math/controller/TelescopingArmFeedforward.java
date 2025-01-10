package frc.lib.math.controller;

import java.util.function.Function;

public class TelescopingArmFeedforward {
    private final Function<Double, Double> ksFunction;
    private final Function<Double, Double> kgFunction;
    private final Function<Double, Double> kvFunction;
    private final Function<Double, Double> kaFunction;

    /**
     * Creates a new TelescopingArmFeedforward with the specified gain functions. The gain functions
     * should calculate the gains at a given position of the arm. Units of the gain values will
     * dictate units of the computed feedforward. Acceleration can be omitted by passing in (x) -> 0
     *
     * @param ksFunction The static gain function.
     * @param kgFunction The gravity gain function.
     * @param kvFunction The velocity gain function.
     * @param kaFunction The acceleration gain function.
     */
    public TelescopingArmFeedforward(
            Function<Double, Double> ksFunction,
            Function<Double, Double> kgFunction,
            Function<Double, Double> kvFunction,
            Function<Double, Double> kaFunction) {
        this.ksFunction = ksFunction;
        this.kgFunction = kgFunction;
        this.kvFunction = kvFunction;
        this.kaFunction = kaFunction;
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param armExtension extension of the arm to be passed to the supplied gain calculator
     *     function.
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor).
     *     If your encoder does not follow this convention, an offset should be added.
     * @param velocityRadPerSec The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(
            double armExtension,
            double positionRadians,
            double velocityRadPerSec,
            double accelRadPerSecSquared) {
        return ksFunction.apply(armExtension) * Math.signum(velocityRadPerSec)
                + kgFunction.apply(armExtension) * Math.cos(positionRadians)
                + kvFunction.apply(armExtension) * velocityRadPerSec
                + kaFunction.apply(armExtension) * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to
     * be zero).
     *
     * @param armExtension extension of the arm to be passed to the supplied gain calculator
     *     function.
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor).
     *     If your encoder does not follow this convention, an offset should be added.
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double armExtension, double positionRadians, double velocity) {
        return calculate(armExtension, positionRadians, velocity, 0);
    }
}
