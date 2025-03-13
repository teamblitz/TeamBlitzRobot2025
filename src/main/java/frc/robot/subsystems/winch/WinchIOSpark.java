package frc.robot.subsystems.winch;

import static frc.robot.Constants.Winch.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class WinchIOSpark implements WinchIO {
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;

    public WinchIOSpark() {
        winchMotor = new SparkMax(ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(CURRENT_LIMIT);

        // TODO: SET POSITION AND VELOCITY CONVERSION FACTOR, LETS DO THIS IN UNITS OF PULLEY
        // ROTATIONS.
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

    // TODO: Implement this, we want to use max motion on the spark
    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
    @Override
    public void setMotionProfile(double position) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void updateInputs(WinchIO.WinchInputs inputs) {
        inputs.rpm = winchMotor.getEncoder().getVelocity();
        inputs.current = winchMotor.getOutputCurrent();
    }
}
