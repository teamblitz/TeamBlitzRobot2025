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
                        (1 / Constants.Elevator.ELEVATOR_GEAR_RATIO) * Constants.Elevator.SPROCKET_CIRCUMFERANCE * 2)
                .velocityConversionFactor(
                        (1 / Constants.Elevator.ELEVATOR_GEAR_RATIO)
                                * (1.0 / 60.0)
                                * Constants.Elevator.SPROCKET_CIRCUMFERANCE * 2);

        // todo fix when we switch back to split control
        sharedConfig.inverted(true);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.apply(sharedConfig);
        followerConfig.follow(left, true);

        left.configure(
                sharedConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        right.configure(
                followerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
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
    public void setFF(double kS, double kV, double kA, double kG) {
        feedforward = new ElevatorFeedforward(kS, kV, kA, kG);
    }

    @Override
    public void setSpeed(double speed) {
        left.set(speed);
    }

    @Override
    public void setVolts(double volts) {
        left.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double position, double velocity, double nextVelocity) {
        leftPid.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculateWithVelocities(velocity, nextVelocity), SparkClosedLoopController.ArbFFUnits.kVoltage);
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
