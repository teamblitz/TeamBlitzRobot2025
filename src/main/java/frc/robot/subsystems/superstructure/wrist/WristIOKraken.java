package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.superstructure.wrist.Wrist;

public class WristIOKraken implements WristIO {
    public final TalonFX wristMotor;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0);

    public WristIOKraken() {
        wristMotor = new TalonFX(0); //TODO SET VAL

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits.withStatorCurrentLimit(Wrist.CURRENT_LIMIT); //TODO CONFIGURE

        //TODO GET GEAR RATIO FIGURED OUT WITH MECH

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        //TODO SET VALUES

        config.Slot0.withKP(Wrist.P); //TODO CONFIGURE

        config.MotorOutput.withInverted(Wrist.wristMotor_INVERT);
        wristMotor.getConfigurator().apply(config);

        wristMotor.setPosition(0);

        ParentDevice.optimizeBusUtilizationForAll(wristMotor);

        BassStatusSignal.setUpdateFrequencyForAll(
                100,
                wristMotor.getPosition(),
                wristMotor.getVelocity(),
                wristMotor.getMotorVoltage(),
                wristMotor.getTorqueCurrent());
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.positionwristMotor = wristMotor.getPosition().getValueAsDouble();

        inputs.velocitywristMotor = wristMotor.getVelocity().getValueAsDouble();

        inputs.voltswristMotor = wristMotor.getMotorVoltage().getValueAsDouble();

        inputs.torqueCurrentwristMotor = wristMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeedwristMotor(double speed) {

        wristMotor.set(speed);
    }

    @Override
    public void setMotionMagicwristMotor(double extension) {
        wristMotor.setControl(motionMagic.withPosition(extension));
    }
}
