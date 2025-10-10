package frc.robot.subsystems.drive.swerveModule.encoder;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class EncoderIOCanCoder implements EncoderIO {

    private final CANcoder encoder;

    public EncoderIOCanCoder(int id, boolean invert) {
        encoder = new CANcoder(id);

        encoder.getConfigurator()
                .apply(
                        new MagnetSensorConfigs()
                                .withSensorDirection(
                                        invert
                                                ? SensorDirectionValue.Clockwise_Positive
                                                : SensorDirectionValue.CounterClockwise_Positive));
    }

    @Override
    public void updateInputs(EncoderIO.EncoderIOInputs inputs) {
        inputs.position = encoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
}
