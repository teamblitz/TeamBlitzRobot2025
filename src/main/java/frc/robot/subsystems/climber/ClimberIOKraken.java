package frc.robot.subsystems.climber;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOKraken implements ClimberIO {
    public final TalonFX leftMotor;
    public final TalonFX rightMotor;
    public TalonFX leader;

    public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    public final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

    public ClimberIOKraken() {
        leftMotor = new TalonFX(LEFT_ID);
        rightMotor = new TalonFX(RIGHT_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake));

        config.CurrentLimits.withStatorCurrentLimit(120);

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

        leader = rightMotor;
    }

    @Override
    public void setSpeed(double speed) {
        leader.set(speed);
    }

    @Override
    public void setVolts(double volts) {
        leader.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setMotionProfile(double position) {
        leader.setControl(motionMagic.withPosition(position));
    }

    public void updateInputs(ClimberInputs inputs) {
        inputs.position = leader.getPosition().getValueAsDouble();
        inputs.rpm = leader.getVelocity().getValueAsDouble();
    }
}
