package frc.robot.subsystems.superstructure.wrist;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;

public class WristIOSpark implements WristIO {

    private SparkMax wristMotor;
    private RelativeEncoder encoder;
    private AbsoluteEncoder absoluteEncoder;
    private SparkClosedLoopController pid;

    boolean useInternalEncoder;

    private ArmFeedforward feedforward;

    public WristIOSpark() {
        wristMotor = new SparkMax(Wrist.CAN_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        encoder = wristMotor.getEncoder();
        absoluteEncoder = wristMotor.getAbsoluteEncoder();
        pid = wristMotor.getClosedLoopController();

        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(Wrist.OPEN_LOOP_RAMP)
                .smartCurrentLimit(Wrist.CURRENT_LIMIT);

        config.encoder
                .positionConversionFactor((1 / Wrist.GEAR_RATIO) * (2 * Math.PI))
                .velocityConversionFactor(
                        (1 / Constants.Wrist.GEAR_RATIO) * (1.0 / 60.0) * (2 * Math.PI));

        config.absoluteEncoder
                .zeroCentered(true)
                .inverted(true)
                .zeroOffset(Wrist.ABS_ENCODER_ZERO)
                .positionConversionFactor((2 * Math.PI))
                .velocityConversionFactor((2 * Math.PI));

        wristMotor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPercent(double percent) {
        wristMotor.set(percent);
    }

    @Override
    public void setSetpoint(double position, double velocity, double nextVelocity) {
        pid.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculateWithVelocities(position, velocity, nextVelocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setPid(double p, double i, double d) {
        wristMotor.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        feedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        wristMotor.configure(
                new SparkMaxConfig()
                        .idleMode(
                                brakeMode
                                        ? SparkBaseConfig.IdleMode.kBrake
                                        : SparkBaseConfig.IdleMode.kCoast),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.volts = wristMotor.getAppliedOutput() * wristMotor.getEncoder().getPosition();
        inputs.positionRadians = encoder.getPosition();
        inputs.velocityRadiansPerSecond = encoder.getVelocity();

        inputs.absoluteEncoderPosition = absoluteEncoder.getPosition();
    }
}
