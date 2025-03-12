package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeIOKraken implements IntakeIO {
    public final TalonFX Intake;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    public IntakeIOKraken() {
        Intake = new TalonFX(0); // TODO SET VALUE

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake));

        config.CurrentLimits.withStatorCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        Intake.setPosition(0);

        ParentDevice.optimizeBusUtilizationForAll(Intake);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                Intake.getPosition(),
                Intake.getVelocity(),
                Intake.getMotorVoltage(),
                Intake.getTorqueCurrent());
    }

    @Override
    public void setSpeed(double speed) {
        Intake.set(speed);
    }
}
