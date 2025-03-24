package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.Constants.Elevator.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// TODO: With the current elevator design it is unlikely that we will want to be able to control
// both sides separately
public class ElevatorIOKraken implements ElevatorIO {
    public final TalonFX leftMotor;
    public final TalonFX rightMotor;
    public TalonFX leader;

    private final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    //    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
    //            .withMotionMagicExpo_kV();

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

    public ElevatorIOKraken() {
        leftMotor = new TalonFX(LEFT_ID);
        rightMotor = new TalonFX(RIGHT_ID);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), false));

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.Feedback.withSensorToMechanismRatio(
                ELEVATOR_GEAR_RATIO / (SPROCKET_CIRCUMFERENCE * 2));
        config.CurrentLimits.withStatorCurrentLimit(120);

        config.MotionMagic.withMotionMagicCruiseVelocity(6).withMotionMagicAcceleration(12);

        config.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
                .withKS(KrakenGains.kS)
                // Originaly in terms of rotations. But because it is a reciprical
                // do the other one
                // ONLY CONVERT KV AND KA, the rest are constant.
                .withKV(metersToRotations(KrakenGains.kV))
                .withKA(metersToRotations(KrakenGains.kA))
                .withKG(KrakenGains.kG)
                .withKP(KrakenGains.kP);

        config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(MAX_POS)
                .withReverseSoftLimitThreshold(MIN_POS);

        config.MotorOutput.withInverted(LEFT_INVERT);
        leftMotor.getConfigurator().apply(config);

        config.MotorOutput.withInverted(RIGHT_INVERT);
        rightMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                leftMotor.getMotorVoltage(),
                leftMotor.getRotorPosition(),
                leftMotor.getRotorVelocity(),
                rightMotor.getMotorVoltage(),
                rightMotor.getRotorPosition(),
                rightMotor.getRotorVelocity());

        /* FOR SYSID */

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
    public void setVolts(double voltage) {
        leader.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setMotionMagic(double extension) {
        leader.setControl(motionMagic.withPosition(extension));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        leftMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        rightMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double metersToRotations(double meters) {
        return meters * ELEVATOR_GEAR_RATIO / (SPROCKET_CIRCUMFERENCE * 2);
    }

    private double rotationsToMeters(double rotations) {
        return rotations * (SPROCKET_CIRCUMFERENCE * 2) / ELEVATOR_GEAR_RATIO;
    }
}
