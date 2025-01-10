package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmIOSpark implements ArmIO {

    private final SparkMax leader;
    private final SparkMax follower;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePid;

    private final DutyCycle rawAbsoluteEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final Encoder quadEncoder;

    boolean useInternalEncoder;
    double quadOffset = 0;
    private PIDController pid;

    public ArmIOSpark(boolean useInternalEncoder) {
        /* Arm Rotation */
        leader = new SparkMax(Arm.ARM_ROT_LEADER, MotorType.kBrushless);
        follower = new SparkMax(Arm.ARM_ROT_FOLLOWER, MotorType.kBrushless);

        this.useInternalEncoder = useInternalEncoder;

        SparkMaxConfig sharedConfig = new SparkMaxConfig();

        sharedConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(Arm.OPEN_LOOP_RAMP)
                .smartCurrentLimit(Arm.CURRENT_LIMIT);

        sharedConfig
                .encoder
                .positionConversionFactor(
                        (1 / Constants.Arm.GEAR_RATIO) // Rotations of motor shaft devided by
                                // reduction = rotations of mechanism
                                * (2 * Math.PI)) // Rotations * 2pi = rotation in radians
                .velocityConversionFactor(
                        (1 / Constants.Arm.GEAR_RATIO) * (1.0 / 60.0) * (2 * Math.PI));

        SparkMaxConfig configLeader = new SparkMaxConfig();
        SparkMaxConfig configFollower = new SparkMaxConfig();

        configLeader.apply(sharedConfig);
        configFollower.apply(sharedConfig);

        configLeader.inverted(false);
        configFollower.follow(leader, true);

        configLeader.softLimit.forwardSoftLimitEnabled(true);
        configLeader.softLimit.reverseSoftLimitEnabled(true);

        configLeader.softLimit.forwardSoftLimit(Arm.MAX_ROT);
        configLeader.softLimit.reverseSoftLimit(Arm.MIN_ROT);

        leader.configure(
                configLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        follower.configure(
                configFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder = leader.getEncoder();
        anglePid = leader.getClosedLoopController();

        setPid(Arm.PidConstants.P, Arm.PidConstants.I, Arm.PidConstants.D);

        rawAbsoluteEncoder = new DutyCycle(new DigitalInput(Arm.ABS_ENCODER));
        absoluteEncoder =
                new DutyCycleEncoder(rawAbsoluteEncoder, 2 * Math.PI, Arm.ABS_ENCODER_OFFSET);
        absoluteEncoder.setInverted(true);
        quadEncoder = new Encoder(Arm.QUAD_A, Arm.QUAD_B, true);

        quadEncoder.setDistancePerPulse(1 / (2048 * 2 * Math.PI));

        seedArmPosition(true);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        // BRUH
        inputs.rotation = getPosition();
        inputs.angularVelocity = angleEncoder.getVelocity();
        inputs.absRotation = getAbsolutePosition();
        inputs.rawAbsEncoder = rawAbsoluteEncoder.getOutput() * (2 * Math.PI);
        inputs.rawAbsEncoderDeg = Math.toDegrees(inputs.rawAbsEncoder);

        inputs.encoderConnected = absoluteEncoder.isConnected();
        inputs.volts = leader.getBusVoltage() * leader.getAppliedOutput();

        inputs.rotationDeg = Units.radiansToDegrees(inputs.rotation);
    }

    /** Updates the arm position setpoint. */
    @Override
    public void setRotationSetpoint(double rot, double arbFFVolts) {
        if (useInternalEncoder) {
            anglePid.setReference(
                    rot,
                    SparkMax.ControlType.kPosition,
                    0,
                    arbFFVolts,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
        } else {
            leader.setVoltage(pid.calculate(getPosition(), rot) + arbFFVolts);
        }
    }

    @Override
    public void setArmSpeed(double percent) {
        leader.set(percent);
    }

    @Override
    public void setArmVolts(double volts) {
        System.out.println(volts);
        leader.setVoltage(volts);
    }

    @Override
    public void seedArmPosition(boolean assumeStarting) {
        if (absoluteEncoder.isConnected()) {
            angleEncoder.setPosition(getAbsolutePosition());
            quadOffset = getAbsolutePosition() - quadEncoder.getDistance();
        } else if (assumeStarting) {
            System.out.printf(
                    "Arm absolute rotation encoder disconnected, assuming position %s%n",
                    Arm.STARTING_POS);
            angleEncoder.setPosition(Arm.STARTING_POS);
        }
    }

    private double getAbsolutePosition() {
        return MathUtil.angleModulus(absoluteEncoder.get());
    }

    // TODO: Function incorrectly uses useInternalEncoder? Will not change to prevent possible
    // conflict
    private double getPosition() {
        return useInternalEncoder
                ? MathUtil.angleModulus(quadEncoder.getDistance() + quadOffset)
                : angleEncoder.getPosition();
    }

    @Override
    public void setBrake(boolean brake) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        leader.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        follower.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPid(double kP, double kI, double kD) {

        SparkMaxConfig wastedMemory = new SparkMaxConfig();

        // TODO: REV LIB 2025: IM QUIRKY AND NEED TO DO A CONFIGURE ROUTINE TO SET PID CONSTANTS.
        wastedMemory.closedLoop.pid(kP, kI, kD);
        leader.configure(
                wastedMemory, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void stop() {
        leader.stopMotor();
        follower.stopMotor();
    }
}
