package frc.robot.subsystems.superstructure.wrist;

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

public class WristIOSpark implements WristIO {

    private SparkMax wristMotor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pid;

    boolean useInternalEncoder;

    public WristIOSpark() {
        wristMotor = new SparkMax(Wrist.CAN_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        encoder = wristMotor.getEncoder();
        pid = wristMotor.getClosedLoopController();

        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
                .openLoopRampRate(Wrist.OPEN_LOOP_RAMP)
                .smartCurrentLimit(Wrist.CURRENT_LIMIT);

        config.encoder
                .positionConversionFactor((1 / Wrist.GEAR_RATIO) * (2 * Math.PI))
                .velocityConversionFactor(
                        (1 / Constants.Wrist.GEAR_RATIO) * (1.0 / 60.0) * (2 * Math.PI));

        wristMotor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPid(double p, double i, double d) {
        wristMotor.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }
}
