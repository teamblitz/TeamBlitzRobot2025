package frc.robot.subsystems.drive.swerveModule.encoder;

import com.reduxrobotics.sensors.canandmag.Canandmag;

public class EncoderIOHelium implements EncoderIO {

    private final Canandmag encoder;

    public EncoderIOHelium(int id, boolean invert) {
        encoder = new Canandmag(id);

        Canandmag.Settings settings = new Canandmag.Settings();
        // For the ctre cancoder, the default input is ccw+ looking at the LED side. However, the
        // helium CANAndCoder is by default CCW+ looking from the face. So we invert the value here
        settings.setInvertDirection(!invert);
        encoder.setSettings(settings);
    }

    @Override
    public void updateInputs(EncoderIO.EncoderIOInputs inputs) {
        inputs.position = encoder.getAbsPosition() * 360;
    }

    public void zeroEncoder() {
        encoder.setPosition(0);
        System.out.println("zeroed");
    }
}
