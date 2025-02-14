package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax motor;

    public IntakeIOSpark() {
        // TODO: Motor needs current limit.
        // https://docs.revrobotics.com/brushless/faq#generally-we-recommend-20a-40a

        SparkMaxConfig config = new SparkMaxConfig();

        config
                .inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT);

        motor = new SparkMax(CAN_ID, SparkLowLevel.MotorType.kBrushless);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();
    }
}
