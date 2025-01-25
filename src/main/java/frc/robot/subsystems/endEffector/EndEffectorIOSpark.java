package frc.robot.subsystems.endEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import static frc.robot.Constants.EndEffector.*;
import com.revrobotics.spark.*;

public class EndEffectorIOSpark implements EndEffectorIO {
    // motor variables go here
    private final SparkMax motor;

    //TODO Do we need a breakBeamSensor?
    public EndEffectorIOSpark() {
        // set up the motor
        motor = new SparkMax(CAN_ID, SparkLowLevel.MotorType.kBrushless);

    }

    @Override
    public void setSpeed(double speed) {
        // run the motor
        motor.set(speed);
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();
    }
}