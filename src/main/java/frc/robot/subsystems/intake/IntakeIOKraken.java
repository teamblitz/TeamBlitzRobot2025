package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.Constants.Intake.*;

public class IntakeIOKraken implements IntakeIO {
    public final TalonFX intake;


    public IntakeIOKraken() {
        intake = new TalonFX(CAN_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake));

        intake.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double speed) {
        intake.set(speed);
    }
}
