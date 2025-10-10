package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Roller.*;

public class RollerIOKraken implements RollerIO {
    // TODO FIX EVERYTHING(I hate imports.)
    public final TalonFX rollerMotor;

    // private final DigitalInput breakBeam;
    public RollerIOKraken() {
        rollerMotor = new TalonFX(ROLLER_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.MotorOutput.withNeutralMode((NeutralModeValue.Coast))
        // .withInverted(
        // INVERTED
        // ? InvertedValue.Clockwise_Positive
        // : InvertedValue.CounterClockwise_Positive);
        // breakBeam = new DigitalInput(0);

        rollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }
}
