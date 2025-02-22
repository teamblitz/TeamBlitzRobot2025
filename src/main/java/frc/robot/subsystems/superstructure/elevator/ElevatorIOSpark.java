package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
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

    private ElevatorFeedforward leftFeedforward = null;
    private ElevatorFeedforward rightFeedforward = null;

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

//        sharedConfig.closedLoop
//                .

        SparkMaxConfig leftConfig = new SparkMaxConfig();

        leftConfig.apply(sharedConfig)
                .inverted(true);


        SparkMaxConfig rightConfig = new SparkMaxConfig();

        rightConfig.apply(sharedConfig);

        left.configure(
                leftConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        right.configure(
                rightConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);


        setPidLeft(
                Elevator.LeftGains.KP,
                Elevator.LeftGains.KI,
                Elevator.LeftGains.KD
        );

        setFFLeft(
                Elevator.LeftGains.KS, Elevator.LeftGains.KG, Elevator.LeftGains.KV, Elevator.LeftGains.KA
        );

        setPidRight(
                Elevator.RightGains.KP,
                Elevator.RightGains.KI,
                Elevator.RightGains.KD
        );

        setFFRight(
                Elevator.RightGains.KS, Elevator.RightGains.KG, Elevator.RightGains.KV, Elevator.RightGains.KA
        );

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

//        new Notifier(
//                () -> {
//                    Logger.recordOutput("elevator/elevatorIOSpark/leftP", left.configAccessor.closedLoop.getP());
//                    Logger.recordOutput("elevator/elevatorIOSpark/rightP", right.configAccessor.closedLoop.getP());
//

//
//                    Logger.recordOutput("elevator/elevatorIOSpark/leftD", left.configAccessor.closedLoop.getD());
//                    Logger.recordOutput("elevator/elevatorIOSpark/rightD", right.configAccessor.closedLoop.getD());
//
//                }
//        ).startPeriodic(5);
    }

    @Override
    public void setPidLeft(double p, double i, double d) {
        System.out.println("Updated left pid");
        left.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPidRight(double p, double i, double d) {
        System.out.println("Updated right pid");
        right.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

    }

    @Override
    public void setFFLeft(double kS, double kG, double kV, double kA) {
        leftFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    }
    @Override
    public void setFFRight(double kS, double kG, double kV, double kA) {
        rightFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
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
                leftFeedforward.calculateWithVelocities(velocity, nextVelocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);

        rightPid.setReference(
                position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                rightFeedforward.calculateWithVelocities(velocity, nextVelocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.velocityLeft = leftEncoder.getVelocity();
        inputs.velocityRight = rightEncoder.getVelocity();

        inputs.voltsLeft = left.getAppliedOutput() * left.getBusVoltage();
        inputs.voltsRight = right.getAppliedOutput() * right.getBusVoltage();

        inputs.appliedOutputLeft = left.getAppliedOutput();
        inputs.appliedOutputRight = right.getAppliedOutput();

        inputs.positionLeft = leftEncoder.getPosition();
        inputs.positionRight = rightEncoder.getPosition();

        // Our limit switches are wired nominally closed (nc), so a value of false means the switch is active
        inputs.bottomLimitSwitch = !bottomLimitSwitch.get();
        inputs.topLimitSwitch = !topLimitSwitch.get();

        inputs.topLimitSwitch = false;
        inputs.bottomLimitSwitch = false;
    }
}
