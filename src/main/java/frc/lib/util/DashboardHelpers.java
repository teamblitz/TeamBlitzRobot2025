package frc.lib.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableType;
import java.util.function.Supplier;

public final class DashboardHelpers {
    private DashboardHelpers() {}

    // I spent way too long on this method
    @SuppressWarnings(value = "cast")
    public static <T> Supplier<T> genericEntrySupplier(
            GenericEntry entry, T defaultValue, NetworkTableType type) {

        return () -> {
            // As long as you don't F up the type and default value type, nothing will go wrong here
            if (entry.get().getType() == type) return (T) entry.get().getValue();
            else return defaultValue;
        };
    }
}
