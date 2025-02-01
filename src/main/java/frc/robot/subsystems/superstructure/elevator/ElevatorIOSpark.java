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
import frc.robot.Constants.Wrist;

public class ElevatorIOSpark implements ElevatorIO {

    private final SparkMax right;
    private final SparkMax left;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final SparkClosedLoopController rightPid;
    private final SparkClosedLoopController leftPid;

    boolean useInternalEncoder;

    public ElevatorIOSpark() {
        right = new SparkMax(Wrist.CAN_ID, SparkLowLevel.MotorType.kBrushless);
        left = new SparkMax(Wrist.CAN_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        rightEncoder = right.getEncoder();
        leftEncoder = left.getEncoder();

        rightPid = right.getClosedLoopController();
        leftPid = left.getClosedLoopController();

        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(Wrist.OPEN_LOOP_RAMP)
                .smartCurrentLimit(Wrist.CURRENT_LIMIT);

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
}
