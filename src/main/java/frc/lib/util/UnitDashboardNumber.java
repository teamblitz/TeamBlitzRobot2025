package frc.lib.util;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Unit;
import frc.lib.util.units.UnitConverter;

public class UnitDashboardNumber extends LoggedTunableNumber {
    private final UnitConverter dashboardToInternal;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public UnitDashboardNumber(String dashboardKey) {
        this(dashboardKey, 0);
    }

    public UnitDashboardNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey, defaultValue, BaseUnits.Value);
    }

    /**
     * Create a new UnitDashboardNumber with SI internal units and a specified dashboard unit.
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     * @param dashboardUnit Unit that will be displayed on the dashboard
     */
    public UnitDashboardNumber(String dashboardKey, double defaultValue, Unit dashboardUnit) {
        this(
                dashboardKey,
                defaultValue,
                UnitConverter.of(dashboardUnit, dashboardUnit.getBaseUnit()));
    }

    /**
     * Create a new LoggedTunableNumber with a default value and dashboard to internal unit
     * converter
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     * @param dashboardToInternal Unit converter from dashboard units to internal units
     */
    public UnitDashboardNumber(
            String dashboardKey, double defaultValue, UnitConverter dashboardToInternal) {
        super(dashboardKey, dashboardToInternal.inverse().applyAsDouble(defaultValue));
        this.dashboardToInternal = dashboardToInternal;
    }

    @Override
    public double get() {
        return dashboardToInternal.applyAsDouble(super.get());
    }
}
