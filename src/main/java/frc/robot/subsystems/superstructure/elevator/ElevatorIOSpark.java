package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {

    private final SparkMax right;
    private final SparkMax left;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final SparkClosedLoopController rightPid;
    private final SparkClosedLoopController leftPid;

    private ElevatorFeedforward feedforward = null;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    boolean useInternalEncoder;

    public ElevatorIOSpark() {
        right = new SparkMax(Elevator.RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
        left = new SparkMax(Elevator.LEFT_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig sharedConfig = new SparkMaxConfig();

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightPid = right.getClosedLoopController();
        leftPid = left.getClosedLoopController();

        topLimitSwitch = new DigitalInput(2);
        bottomLimitSwitch = new DigitalInput(1);
        sharedConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(Elevator.OPEN_LOOP_RAMP)
                .smartCurrentLimit(Elevator.CURRENT_LIMIT);

        sharedConfig
                .encoder
                .positionConversionFactor(
                        (1 / Constants.Elevator.ELEVATOR_GEAR_RATIO)
                                * Constants.Elevator.SPROCKET_CIRCUMFERANCE
                                * 2)
                .velocityConversionFactor(
                        (1 / Constants.Elevator.ELEVATOR_GEAR_RATIO)
                                * (1.0 / 60.0)
                                * Constants.Elevator.SPROCKET_CIRCUMFERANCE
                                * 2);

        SparkMaxConfig leftConfig = new SparkMaxConfig();

        leftConfig.apply(sharedConfig)
                .inverted(true);


        SparkMaxConfig rightConfig = new SparkMaxConfig();

//        rightConfig.follow(left, true);


        rightConfig.apply(sharedConfig);

        left.configure(
                leftConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        right.configure(
                rightConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        setPid(
                Elevator.Gains.KP,
                Elevator.Gains.KI,
                Elevator.Gains.KD
        );

        setFF(
                Elevator.Gains.KS, Elevator.Gains.KG, Elevator.Gains.KV, Elevator.Gains.KA
        );
    }

    @Override
    public void setPid(double p, double i, double d) {
        right.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        left.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        SparkMaxConfig brakeConfig = new SparkMaxConfig();

        brakeConfig.idleMode(
                brakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        left.configure(
                brakeConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        right.configure(
                brakeConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        left.set(speed);
        right.set(speed);
    }

    @Override
    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }

    @Override
    public void setVolts(double volts) {
        left.setVoltage(volts);
        right.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double position, double velocity, double nextVelocity) {
        leftPid.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculateWithVelocities(velocity, nextVelocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);

        rightPid.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculateWithVelocities(velocity, nextVelocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.velocityLeft = leftEncoder.getVelocity();
        inputs.velocityRight = rightEncoder.getVelocity();

        inputs.voltsLeft = left.getAppliedOutput() * left.getBusVoltage();
        inputs.voltsRight = right.getAppliedOutput() * right.getBusVoltage();

        inputs.positionLeft = leftEncoder.getPosition();
        inputs.positionRight = rightEncoder.getPosition();
    }
}
