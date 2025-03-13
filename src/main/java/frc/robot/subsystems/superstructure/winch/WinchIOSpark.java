package frc.robot.subsystems.superstructure.winch;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.Winch;
import frc.robot.subsystems.intake.IntakeIO;

import static frc.robot.Constants.Wrist.GEAR_RATIO;

public class WinchIOSpark implements WinchIO{
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;

    public WinchIOSpark () {
        winchMotor = new SparkMax(Winch.WINCH_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        winchEncoder = winchMotor.getEncoder();

        winchMotor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

    }
    @Override
    public void setSpeed(double speed) {
        winchMotor.set(speed);
    }
    @Override
    public void updateInputs(WinchIO.WinchInputs inputs) {
        inputs.rpm = winchMotor.getEncoder().getVelocity();
        inputs.current = winchMotor.getOutputCurrent();
    }
}
