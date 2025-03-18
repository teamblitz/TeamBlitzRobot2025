package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.Constants.Elevator.*;

// TODO: With the current elevator design it is unlikely that we will want to be able to control
// both sides separately
public class ElevatorIOKraken implements ElevatorIO {
    public final TalonFX leftMotor;
    public final TalonFX rightMotor;
    public TalonFX leader;

    public final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0);

    public ElevatorIOKraken() {
        leftMotor = new TalonFX(LEFT_ID);
        rightMotor = new TalonFX(RIGHT_ID);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), false));

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.Feedback.withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO / SPROCKET_CIRCUMFERENCE);

        config.CurrentLimits.withStatorCurrentLimit(120);

        config.MotionMagic.withMotionMagicCruiseVelocity(.5).withMotionMagicAcceleration(1);

        config.MotorOutput.withInverted(LEFT_INVERT);
        leftMotor.getConfigurator().apply(config);

        config.MotorOutput.withInverted(RIGHT_INVERT);
        rightMotor.getConfigurator().apply(config);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

        leader = rightMotor;
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
        leader.set(speed);
    }

    @Override
    public void setMotionMagic(double extension) {
        leader.setControl(motionMagic.withPosition(extension));
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        leftMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        rightMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
