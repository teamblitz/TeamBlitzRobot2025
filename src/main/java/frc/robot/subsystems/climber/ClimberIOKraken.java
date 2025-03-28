package frc.robot.subsystems.climber;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class ClimberIOKraken implements ClimberIO {
    public final TalonFX leftMotor;
    public final TalonFX rightMotor;
    public TalonFX leader;

    public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    public final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

    public final DutyCycle rawAbsEncoder;
    public final DutyCycleEncoder absEncoder;

    public ClimberIOKraken() {
        leftMotor = new TalonFX(LEFT_ID);
        rightMotor = new TalonFX(RIGHT_ID);

        rawAbsEncoder = new DutyCycle(new DigitalInput(ABS_ENCODER_DIO_PORT));
        absEncoder = new DutyCycleEncoder(rawAbsEncoder, 2 * Math.PI, ABS_ENCODER_ZERO);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake));
        config.CurrentLimits.withStatorCurrentLimit(95);
        config.Feedback.withSensorToMechanismRatio(CLIMBER_GEAR_RATIO / (2 * Math.PI));

        config.Slot0.withKS(UnloadedGains.KS)
                .withKV(UnloadedGains.KV)
                .withKA(UnloadedGains.KA)
                .withKP(UnloadedGains.KP);

        config.MotionMagic.withMotionMagicCruiseVelocity(MAX_VEL_UNLOADED)
                .withMotionMagicAcceleration(MAX_ACCEL_UNLOADED);

        config.Slot1.withKS(LoadedGains.KS)
                .withKV(LoadedGains.KV)
                .withKA(LoadedGains.KA)
                .withKP(LoadedGains.KP);

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

        leader = rightMotor;

        leader.setPosition(STARTING_POSITION);

        new Trigger(absEncoder::isConnected).debounce(1).onTrue(
                        Commands.runOnce(
                                () -> {
                                    if (absEncoder.isConnected())
                                        leader.setPosition(getAbsPosition());
                                })
                .ignoringDisable(true)
                                .withName("climber/seedPosition"));
    }

    private double getAbsPosition() {
        return absEncoder.get();
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
        inputs.velocity = leader.getVelocity().getValueAsDouble();

        inputs.absolutePosition = getAbsPosition();

        Logger.recordOutput("climber/climberIOKraken/rawAbsEncoder", rawAbsEncoder.getOutput());
    }

    @Override
    public void setBrakeMode(boolean brake) {
        leftMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        rightMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
