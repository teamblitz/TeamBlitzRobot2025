package frc.lib.util.units;

import edu.wpi.first.units.Unit;

import lombok.NonNull;

import java.util.function.DoubleUnaryOperator;

public class UnitConverter implements DoubleUnaryOperator {
    Unit from;
    Unit to;

    public UnitConverter(@NonNull Unit from, @NonNull Unit to) {
        if (from.getBaseUnit() != to.getBaseUnit()) {
            throw new IllegalArgumentException("Unit " + from + " is not compatible with " + to);
        }
    }

    public UnitConverter inverse() {
        return new UnitConverter(to, from);
    }

    @Override
    public double applyAsDouble(double operand) {
        return to.fromBaseUnits(from.toBaseUnits(operand));
    }

    public static UnitConverter of(Unit from, Unit to) {
        return new UnitConverter(from, to);
    }
}
