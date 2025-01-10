package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeIOSpark implements IntakeIO {

    private final SparkMax motor;

    private final DigitalInput breakBeamSensor;

    public IntakeIOSpark() {
        motor = new SparkMax(Constants.Intake.Spark.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        breakBeamSensor = new DigitalInput(9);

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(Constants.Intake.CURRENT_LIMIT)
                .idleMode(SparkBaseConfig.IdleMode.kCoast);

        motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();

        inputs.breakBeam = breakBeamSensor.get();
    }
}
