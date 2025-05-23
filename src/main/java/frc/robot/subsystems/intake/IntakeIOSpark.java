package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.monitor.HardwareWatchdog;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax motor;

    private final DigitalInput breakBeam;

    public IntakeIOSpark() {
        SparkMaxConfig config = new SparkMaxConfig();

        breakBeam = new DigitalInput(2);

        config.inverted(INVERTED).smartCurrentLimit(CURRENT_LIMIT);

        motor = new SparkMax(CAN_ID, SparkLowLevel.MotorType.kBrushless);

        motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        HardwareWatchdog.getInstance().registerSpark(motor, this.getClass());
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();

        inputs.breakBeam = breakBeam.get();
    }
}
