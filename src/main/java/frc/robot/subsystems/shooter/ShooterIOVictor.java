package frc.robot.subsystems.shooter;

// NOT TALONS
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class ShooterIOVictor implements ShooterIO {
    private final WPI_VictorSPX top;
    private final WPI_VictorSPX bottom;

    public ShooterIOVictor() {
        top = new WPI_VictorSPX(Constants.Shooter.Talon.TALON_TOP);
        bottom = new WPI_VictorSPX(Constants.Shooter.Talon.TALON_BOTTOM);
    }

    @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {}
}
