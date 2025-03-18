package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.Constants.Wrist.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.Constants.Wrist.*;

public class WristIOKraken implements WristIO {
    public TalonFX wristMotor;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    public WristIOKraken() {
        wristMotor = new TalonFX(CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT);

        config.Feedback.withSensorToMechanismRatio(WRIST_GEAR_RATIO / (2 * Math.PI));


        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        // TODO SET VALUES

        //            config.Slot0.withKP(Wrist.P); //TODO CONFIGURE

        config.MotorOutput.withInverted(
                INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive);
        wristMotor.getConfigurator().apply(config);

        wristMotor.setPosition(0);

        ParentDevice.optimizeBusUtilizationForAll(wristMotor);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wristMotor.getPosition(),
                wristMotor.getVelocity(),
                wristMotor.getMotorVoltage(),
                wristMotor.getTorqueCurrent());
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.positionRadians = wristMotor.getPosition().getValueAsDouble();

        inputs.velocityRadiansPerSecond = wristMotor.getVelocity().getValueAsDouble();

        inputs.volts = wristMotor.getMotorVoltage().getValueAsDouble();

        inputs.torqueCurrent = wristMotor.getTorqueCurrent().getValueAsDouble();
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
}
