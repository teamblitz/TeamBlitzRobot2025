package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.Constants.Wrist.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.Wrist.*;

public class WristIOKraken implements WristIO {
    private final TalonFX wristMotor;

    private final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

    private final Canandmag absoluteEncoder;

    public WristIOKraken() {
        wristMotor = new TalonFX(CAN_ID);
        absoluteEncoder = new Canandmag(ABS_ENCODER_ID);

        CanandmagSettings canandmagSettings = new CanandmagSettings();
        canandmagSettings.setInvertDirection(true);

        absoluteEncoder.setSettings(canandmagSettings);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);
        config.Feedback.withSensorToMechanismRatio(WRIST_GEAR_RATIO / (2 * Math.PI));
        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        config.MotorOutput.withInverted(
                INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive);
        wristMotor.getConfigurator().apply(config);


        Commands.sequence(
            Commands.waitSeconds(2),
                Commands.runOnce(
                        () -> {
                            if (absoluteEncoder.isConnected()) wristMotor.setPosition(absoluteEncoder.getAbsPosition() * Math.PI * 2);
                            else wristMotor.setPosition(Math.toRadians(90));
                        })).ignoringDisable(true).schedule();

//        ParentDevice.optimizeBusUtilizationForAll(wristMotor);
//
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wristMotor.getRotorPosition(),
                wristMotor.getRotorVelocity(),
                wristMotor.getMotorVoltage());


    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.positionRadians = wristMotor.getPosition().getValueAsDouble();
        inputs.velocityRadiansPerSecond = wristMotor.getVelocity().getValueAsDouble();
        inputs.volts = wristMotor.getMotorVoltage().getValueAsDouble();
        inputs.torqueCurrent = wristMotor.getTorqueCurrent().getValueAsDouble();

        inputs.absoluteEncoderPosition = 2 * Math.PI * absoluteEncoder.getAbsPosition();

        wristMotor.getRotorPosition().getValueAsDouble();
        wristMotor.getRotorVelocity().getValueAsDouble();
    }

    @Override
    public void setPercent(double percent) {
        wristMotor.set(percent);
    }

    @Override
    public void setSetpoint(double position, double velocity, double acceleration) {
        throw new UnsupportedOperationException("Not supported yet.");
        //            wristMotor.setControl(motionMagic.withPosition(extension));
    }

    @Override
    public void setVolts(double voltage) {
        wristMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        wristMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
