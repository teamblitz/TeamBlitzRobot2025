package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

public class ElevatorIOSpark implements ElevatorIO {

    private final SparkMax right;
    private final SparkMax left;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final SparkClosedLoopController rightPid;
    private final SparkClosedLoopController leftPid;

    boolean useInternalEncoder;

    public ElevatorIOSpark() {
        right = new SparkMax(Elevator.RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
        left = new SparkMax(Elevator.LEFT_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightPid = right.getClosedLoopController();
        leftPid = left.getClosedLoopController();

        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(Elevator.OPEN_LOOP_RAMP)
                .smartCurrentLimit(Elevator.CURRENT_LIMIT);

        config.encoder
                .positionConversionFactor(
                        (1 / Constants.Elevator.ELEVATOR_GEAR_RATIO) * (2 * Math.PI))
                .velocityConversionFactor(
                        (1 / Constants.Elevator.ELEVATOR_GEAR_RATIO)
                                * (1.0 / 60.0)
                                * (2 * Math.PI));

        right.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        config.inverted(true);

        left.configure(
                config,
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
    public void setSpeedLeft(double speed) {
        left.set(speed);
    }

    @Override
    public void setSpeedRight(double speed) {
        right.set(speed);
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
