package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;

public class ClimberIOKraken implements ClimberIO {
    public final TalonFX Climber;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    public ClimberIOKraken() {
        Climber = new TalonFX(0); //TODO Put Correct Value

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode((NeutralModeValue.Brake));

        config.CurrentLimits.withStatorCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);
        //TODO SET ABOVE VALUES

        Climber.setPosition(0);

        ParentDevice.optimizeBusUtilizationForAll(Climber);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                Climber.getPosition(),
                Climber.getVelocity(),
                Climber.getMotorVoltage(),
                Climber.getTorqueCurrent());
    }

    @Override
    public void setSpeed(double speed) {Climber.set(speed);}
}
