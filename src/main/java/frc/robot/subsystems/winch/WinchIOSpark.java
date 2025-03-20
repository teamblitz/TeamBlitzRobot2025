package frc.robot.subsystems.winch;

import static frc.robot.Constants.Winch.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class WinchIOSpark implements WinchIO {
    private final SparkMax motor;
    private final SparkClosedLoopController pid;
    private final RelativeEncoder encoder;

    public WinchIOSpark() {
        motor = new SparkMax(ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(CURRENT_LIMIT);

        config.encoder.positionConversionFactor(1 / WINCH_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / WINCH_GEAR_RATIO * 60);

//        config.closedLoop.

        encoder = motor.getEncoder();
        pid = motor.getClosedLoopController();

        motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setMotionProfile(double position) {
        pid.setReference(
                position,
                SparkBase.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(WinchIO.WinchInputs inputs) {
        inputs.velocity = encoder.getVelocity();
        inputs.position = encoder.getPosition();
        inputs.current = motor.getOutputCurrent();
    }

    @Override
    public void setPid(double p, double i, double d) {
        motor.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setMotionProfilePrams(double maxVel, double maxAccel) {
        motor.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().apply(new MAXMotionConfig().maxVelocity(maxVel).maxAcceleration(maxAccel))),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters
        );
    }
}
