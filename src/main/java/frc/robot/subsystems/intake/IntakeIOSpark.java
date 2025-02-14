package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.revrobotics.spark.*;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax motor;

    public IntakeIOSpark() {
        // TODO: Motor needs current limit.
        // https://docs.revrobotics.com/brushless/faq#generally-we-recommend-20a-40a
        motor = new SparkMax(CAN_ID, SparkLowLevel.MotorType.kBrushless);
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
