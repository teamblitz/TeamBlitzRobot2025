package frc.robot.subsystems.endEffector;

import static frc.robot.Constants.EndEffector.*;

import com.revrobotics.spark.*;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax motor;

    // TODO Do we need a breakBeamSensor?
    public EndEffectorIOSpark() {
        // TODO: Motor needs current limit.
        // https://docs.revrobotics.com/brushless/faq#generally-we-recommend-20a-40a
        motor = new SparkMax(CAN_ID, SparkLowLevel.MotorType.kBrushless);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();
    }
}
