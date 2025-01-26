package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.Elevator;

// TODO: With the current elevator design it is unlikely that we will want to be able to control
// both sides separately
public class ElevatorIOKraken implements ElevatorIO {
    public final TalonFX left;
    public final TalonFX right;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0); // do we impliment the same way?

    public ElevatorIOKraken() {
        left = new TalonFX(0); // TODO set val
        right = new TalonFX(0); // TODO set val

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits.withStatorCurrentLimit(Elevator.CURRENT_LIMIT); // TODO Configure

        // TODO get gear ratio figured out with mech

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        config.Slot0.withKP(Elevator.P); // TODO configure

        config.MotorOutput.withInverted(Elevator.LEFT_INVERT); // TODO configure
        left.getConfigurator().apply(config);

        config.MotorOutput.withInverted(Elevator.RIGHT_INVERT); // TODO configure
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
    public void updateInputs(ElevatorIOInputs inputs) {
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
