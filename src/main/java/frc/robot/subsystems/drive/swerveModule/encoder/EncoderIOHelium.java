package frc.robot.subsystems.drive.swerveModule.encoder;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import frc.lib.monitor.HardwareWatchdog;

public class EncoderIOHelium implements EncoderIO {

    private final Canandmag encoder;

    public EncoderIOHelium(int id, boolean invert) {
        encoder = new Canandmag(id);

        CanandmagSettings settings = new CanandmagSettings();
        // For the ctre cancoder, the default input is ccw+ looking at the LED side. However, the
        // helium CANAndCoder is by default CCW+ looking from the face. So we invert the value here
        settings.setInvertDirection(!invert);
        encoder.setSettings(settings);

        HardwareWatchdog.getInstance().registerReduxDevice(encoder, this.getClass());
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
