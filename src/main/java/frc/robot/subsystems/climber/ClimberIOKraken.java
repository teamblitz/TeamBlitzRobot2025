package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.Climber;

public class ClimberIOKraken implements ClimberIO {

    public final TalonFX left;
    public final TalonFX right;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    public ClimberIOKraken() {
        left = new TalonFX(Climber.LEFT_MOTOR_ID); // 17
        right = new TalonFX(Climber.RIGHT_MOTOR_ID); // 18

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits.withStatorCurrentLimit(Climber.CURRENT_LIMIT);

        config.Feedback
                .withSensorToMechanismRatio( // how many sensor rotations per mechanism rotation.
                        Climber.GEAR_RATIO * (1 / (Math.PI * Climber.SPOOL_DIAMETER)));

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        config.Slot0.withKP(Climber.P);

        config.MotorOutput.withInverted(Climber.LEFT_INVERT);
        left.getConfigurator().apply(config);

        config.MotorOutput.withInverted(Climber.RIGHT_INVERT);
        right.getConfigurator().apply(config);

        left.setPosition(0);
        right.setPosition(0);

        ParentDevice.optimizeBusUtilizationForAll(left, right);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                left.getPosition(),
                left.getVelocity(),
                left.getMotorVoltage(),
                left.getTorqueCurrent(),
                right.getPosition(),
                right.getVelocity(),
                right.getMotorVoltage(),
                right.getTorqueCurrent());
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.positionLeft = left.getPosition().getValueAsDouble();
        inputs.positionRight = right.getPosition().getValueAsDouble();

        inputs.velocityLeft = left.getVelocity().getValueAsDouble();
        inputs.velocityRight = right.getVelocity().getValueAsDouble();

        inputs.voltsLeft = left.getMotorVoltage().getValueAsDouble();
        inputs.voltsRight = right.getMotorVoltage().getValueAsDouble();

        inputs.torqueCurrentLeft = left.getTorqueCurrent().getValueAsDouble();
        inputs.torqueCurrentRight = right.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeedLeft(double speed) {
        left.set(speed);
    }

    @Override
    public void setSpeedRight(double speed) {
        right.set(speed);
    }

    @Override
    public void setMotionMagicLeft(double extension) {
        left.setControl(motionMagic.withPosition(extension));
    }

    @Override
    public void setMotionMagicRight(double extension) {
        right.setControl(motionMagic.withPosition(extension));
    }
}
