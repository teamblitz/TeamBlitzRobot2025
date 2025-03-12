package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.Elevator;

// TODO: With the current elevator design it is unlikely that we will want to be able to control
// both sides separately
public class ElevatorIOKraken implements ElevatorIO {
    public final TalonFX leftMotor;
    public final TalonFX rightMotor;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0); // do we impliment the same way?

    public ElevatorIOKraken() {
        leftMotor = new TalonFX(1); // TODO set val
        rightMotor = new TalonFX(2); // TODO set val

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), false));

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits.withStatorCurrentLimit(Elevator.CURRENT_LIMIT); // TODO Configure

        // TODO get gear ratio figured out with mech

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        config.Slot0.withKP(Elevator.P); // TODO configure

        config.MotorOutput.withInverted(Elevator.LEFT_INVERT); // TODO configure
        leftMotor.getConfigurator().apply(config);

        config.MotorOutput.withInverted(Elevator.RIGHT_INVERT); // TODO configure
        rightMotor.getConfigurator().apply(config);

        TalonFXConfiguration shared_config = new TalonFXConfiguration();

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);

        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                leftMotor.getPosition(),
                leftMotor.getVelocity(),
                leftMotor.getMotorVoltage(),
                leftMotor.getTorqueCurrent(),
                rightMotor.getPosition(),
                rightMotor.getVelocity(),
                rightMotor.getMotorVoltage(),
                rightMotor.getTorqueCurrent());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionLeft = leftMotor.getPosition().getValueAsDouble();
        inputs.positionRight = rightMotor.getPosition().getValueAsDouble();

        inputs.velocityLeft = leftMotor.getVelocity().getValueAsDouble();
        inputs.velocityRight = rightMotor.getVelocity().getValueAsDouble();

        inputs.voltsLeft = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.voltsRight = rightMotor.getMotorVoltage().getValueAsDouble();

        inputs.torqueCurrentLeft = leftMotor.getTorqueCurrent().getValueAsDouble();
        inputs.torqueCurrentRight = rightMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    public void setMotionMagic(double extension) {
        rightMotor.setControl(motionMagic.withPosition(extension));
    }
}
