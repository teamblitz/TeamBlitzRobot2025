package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ShooterIOSpark implements ShooterIO {

    private final SparkMax top;
    private final SparkMax bottom;

    private final SparkClosedLoopController pidTop;
    private final SparkClosedLoopController pidBottom;

    private SimpleMotorFeedforward feedforwardTop;
    private SimpleMotorFeedforward feedforwardBottom;

    public ShooterIOSpark() {
        top = new SparkMax(Constants.Shooter.Spark.SPARK_TOP, SparkLowLevel.MotorType.kBrushless);
        bottom =
                new SparkMax(
                        Constants.Shooter.Spark.SPARK_BOTTOM, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(Constants.Shooter.CURRENT_LIMIT)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .openLoopRampRate(.5)
                .closedLoopRampRate(.2);

        config.encoder
                .velocityConversionFactor(Constants.Shooter.Spark.VELOCITY_FACTOR_RPM_TO_MPS)
                .positionConversionFactor(Constants.Shooter.Spark.POSITION_FACTOR_ROT_TO_M);

        top.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        bottom.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        pidTop = top.getClosedLoopController();
        pidBottom = bottom.getClosedLoopController();

        setTopPid(
                Constants.Shooter.Spark.PID_TOP_P,
                Constants.Shooter.Spark.PID_TOP_I,
                Constants.Shooter.Spark.PID_TOP_D);

        setBottomPid(
                Constants.Shooter.Spark.PID_BOTTOM_P,
                Constants.Shooter.Spark.PID_BOTTOM_I,
                Constants.Shooter.Spark.PID_BOTTOM_D);

        feedforwardTop =
                new SimpleMotorFeedforward(
                        Constants.Shooter.Spark.FF_TOP_KS,
                        Constants.Shooter.Spark.FF_TOP_KV,
                        Constants.Shooter.Spark.FF_TOP_KA);

        feedforwardBottom =
                new SimpleMotorFeedforward(
                        Constants.Shooter.Spark.FF_BOTTOM_KS,
                        Constants.Shooter.Spark.FF_BOTTOM_KV,
                        Constants.Shooter.Spark.FF_BOTTOM_KA);

        SmartDashboard.putNumber("MAX SHOOT Top", feedforwardTop.maxAchievableVelocity(12, 0));
        SmartDashboard.putNumber(
                "MAX SHOOT Bottom", feedforwardBottom.maxAchievableVelocity(12, 0));
    }

    @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }

    @Override
    public void setVolts(double volts) {
        top.setVoltage(volts);
        bottom.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double velocity) {
        pidTop.setReference(
                velocity,
                SparkBase.ControlType.kVelocity,
                0,
                feedforwardTop.calculate(velocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);
        pidBottom.setReference(
                velocity,
                SparkBase.ControlType.kVelocity,
                0,
                feedforwardBottom.calculate(velocity),
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rpmTop = top.getEncoder().getVelocity();
        inputs.rpmBottom = bottom.getEncoder().getVelocity();

        inputs.currentTop = top.getOutputCurrent();
        inputs.currentBottom = bottom.getOutputCurrent();
    }

    @Override
    public void setTopPid(double p, double i, double d) {
        top.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setBottomPid(double p, double i, double d) {
        bottom.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void setTopFF(double kS, double kV, double kA) {
        feedforwardTop = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void setBottomFF(double kS, double kV, double kA) {
        feedforwardBottom = new SimpleMotorFeedforward(kS, kV, kA);
    }
}
