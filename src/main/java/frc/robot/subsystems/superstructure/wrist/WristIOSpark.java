package frc.robot.subsystems.superstructure.wrist;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.Wrist.*;

public class WristIOSpark implements WristIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController pid;

    boolean useInternalEncoder;

    private ArmFeedforward feedforward;

    public WristIOSpark() {
        motor = new SparkMax(CAN_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        encoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
        pid = motor.getClosedLoopController();

        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(OPEN_LOOP_RAMP)
                .smartCurrentLimit(CURRENT_LIMIT);

        config.softLimit
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(FORWARD_SOFT_LIMIT)
                .reverseSoftLimit(REVERSE_SOFT_LIMIT);

        config.encoder
                .positionConversionFactor((1 / GEAR_RATIO) * (2 * Math.PI))
                .velocityConversionFactor(
                        (1 / GEAR_RATIO) * (1.0 / 60.0) * (2 * Math.PI));

        config.absoluteEncoder
                .zeroCentered(true)
                .inverted(true)
                .zeroOffset(
                        Units.radiansToRotations(
                                MathUtil.inputModulus(ABS_ENCODER_ZERO, 0, 2 * Math.PI)))
                .positionConversionFactor((2 * Math.PI))
                .velocityConversionFactor((2 * Math.PI));

        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);


        setPid(
                PidGains.KP,
                PidGains.KI,
                PidGains.KD
        );

        setFF(
                WristGains.KS, WristGains.KG, WristGains.KV, WristGains.KA
        );

        Commands.waitSeconds(.25).andThen(Commands.runOnce(() -> encoder.setPosition(absoluteEncoder.getPosition())).ignoringDisable(true)).schedule();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double position, double velocity, double nextVelocity) {
//        System.out.println("Wrist IO Spark setpoint request: " + position + " " + velocity + " " + nextVelocity);

        // TODO: WPILIB BUG MEANS THAT VERY SMALL DIFFERENCES BETWEEN V(t) and V(t+1) can result in a infinite loop
//        double arbFF = feedforward.calculateWithVelocities(position, velocity, nextVelocity);

        double arbFF = feedforward.calculate(position, velocity);

//        System.out.println("Wrist IO Spark setpoint feedforward: " + arbFF);

        REVLibError errorCode = pid.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage);

//        System.out.println("Wrist IO Spark setpoint error code: " + errorCode);

    }

    @Override
    public void setPid(double p, double i, double d) {
        motor.configure(
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
        motor.configure(
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
        inputs.volts = motor.getAppliedOutput() * motor.getEncoder().getPosition();
        inputs.positionRadians = encoder.getPosition();
        inputs.velocityRadiansPerSecond = encoder.getVelocity();

        inputs.absoluteEncoderPosition = absoluteEncoder.getPosition();
    }
}
