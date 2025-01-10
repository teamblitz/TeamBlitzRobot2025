package frc.robot.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

@SuppressWarnings("unused")
public class GyroIONavx implements GyroIO {

    /*
     * Axis       Orientation       Linear motion           Rotational Motion
     * X (Pitch)  Left/Right        – Left / + Right        + Tilt Backwards
     * Y (Roll)   Forward/Backward  + Forward / – Backward  + Roll Left
     * Z (Yaw)    Up/Down           + Up / – Down           + Clockwise/ – Counterclockwise
     *
     * https://pdocs.kauailabs.com/navx-mxp/wp-content/uploads/2020/09/navx2-mxp_robotics_navigation_sensor_user_guide-8.pdf
     */
    private final AHRS gyro;

    public GyroIONavx() {
        gyro = new AHRS();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // The navx yaw pitch roll values need to be inverted to make them use the same coordinate
        // system as us.
        // WPILIB's coordinate system is X forward Y to the left and Z up
        // Navx is y forward x to the right, and z up.
        inputs.yaw =
                (-gyro.getAngle() + 180)
                        % 360; // Maybe this should be get yaw once we confirm the behavior of
        // the pigeon
        inputs.pitch = -gyro.getPitch();
        inputs.roll = -gyro.getRoll();
        inputs.yawRate = -gyro.getRate();
        inputs.pitchRate = -gyro.getRawGyroX();
        inputs.rollRate = -gyro.getRawGyroY();
        inputs.connected = gyro.isConnected();
    }
}
