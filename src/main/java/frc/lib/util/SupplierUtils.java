package frc.lib.util;

import java.util.function.*;

public final class SupplierUtils {
    public static <T> Supplier<T> apply(Function<T, T> function, Supplier<T> supplier) {
        return () -> function.apply(supplier.get());
    }

    public static DoubleSupplier apply(DoubleUnaryOperator function, DoubleSupplier supplier) {
        return () -> function.applyAsDouble(supplier.getAsDouble());
    }

    public static DoubleSupplier toRadians(DoubleSupplier supplier) {
        return apply(Math::toRadians, supplier);
    }

    public static DoubleSupplier toDegrees(DoubleSupplier supplier) {
        return apply(Math::toDegrees, supplier);
    }

    private SupplierUtils() {}
}
