package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    public GyroIOSim(Supplier<ChassisSpeeds> speedsSupplier) {
        chassisSpeeds = speedsSupplier;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawRate = Units.radiansToDegrees(chassisSpeeds.get().omegaRadiansPerSecond);
        inputs.yaw += inputs.yawRate * Constants.LOOP_PERIOD_SEC;
    }
}
