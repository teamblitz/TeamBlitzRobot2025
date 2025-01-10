package frc.lib.util;

import edu.wpi.first.units.measure.BaseUnits;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.units.measure.measure.Voltage;

import java.util.function.DoubleUnaryOperator;

public class UnitDashboardNumber extends LoggedTunableNumber{
    private final Unit displalyUnit;



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
     * Create a new LoggedTunableNumber with a default value and display unit
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     * 
     */
    public UnitDashboardNumber(String dashboardKey, double defaultValue, Unit displalyUnit) {
        super(dashboardKey, realToDisplay.applyAsDouble(defaultValue));
        this.realToDisplay = realToDisplay;
        this.displayToReal = displayToReal;
    }

    @Override
    public double get() {
        displalyUnit.toBaseUnits(super.get());
        return displayToReal.applyAsDouble(super.get());
    }

    /**
     * Builds a UnitDashboardNumber with internal unit of radians and a display unit of degrees
     */
    public static UnitDashboardNumber radiansDegrees(String dashboardKey, double defaultValue) {
        return new UnitDashboardNumber(dashboardKey, defaultValue, Units::radiansToDegrees, Units::degreesToRadians);
    }
}
