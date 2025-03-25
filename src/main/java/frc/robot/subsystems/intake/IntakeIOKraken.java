package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOKraken implements IntakeIO {
    public final TalonFX intake;

    private final DigitalInput breakBeam;


    public IntakeIOKraken() {
        intake = new TalonFX(CAN_ID); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.MotorOutput.withNeutralMode((NeutralModeValue.Coast))
                .withInverted(INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        breakBeam = new DigitalInput(0);


        intake.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double speed) {
        intake.set(speed);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.breakBeam = !breakBeam.get();
    }
}
