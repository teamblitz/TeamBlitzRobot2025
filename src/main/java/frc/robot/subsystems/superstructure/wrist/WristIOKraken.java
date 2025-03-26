package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.Constants.Wrist.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Wrist.*;

public class WristIOKraken implements WristIO {
    private final TalonFX wristMotor;

    private final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
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
        config.CurrentLimits.withStatorCurrentLimit(80);
        config.Feedback.withSensorToMechanismRatio(WRIST_GEAR_RATIO);


        config.MotionMagic.withMotionMagicCruiseVelocity(Units.radiansToRotations(MAX_VELOCITY))
                .withMotionMagicAcceleration(Units.radiansToRotations(MAX_ACCEL));

        config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(KrakenGains.KS)
                .withKV(KrakenGains.KV)
                .withKA(KrakenGains.KA)
                .withKG(KrakenGains.KG)
                .withKP(KrakenGains.KP);
//                .withKD(KrakenGains.KD);

        config.MotorOutput.withInverted(
                INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive);
        wristMotor.getConfigurator().apply(config);

        config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(MAX_POS)
                .withReverseSoftLimitThreshold(MIN_POS);

        Commands.sequence(
                        Commands.waitSeconds(2),
                        Commands.runOnce(
                                () -> {
                                    if (absoluteEncoder.isConnected())
                                        wristMotor.setPosition(getAbsPosition() / (2 * Math.PI));
                                    else wristMotor.setPosition(Units.degreesToRotations(90));
                                }))
                .ignoringDisable(true)
                .schedule();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wristMotor.getRotorPosition(),
                wristMotor.getRotorVelocity(),
                wristMotor.getMotorVoltage());
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.positionRadians = 2 * Math.PI * wristMotor.getPosition().getValueAsDouble();
        inputs.velocityRadiansPerSecond = 2 * Math.PI * wristMotor.getVelocity().getValueAsDouble();
        inputs.volts = wristMotor.getMotorVoltage().getValueAsDouble();

        inputs.absoluteEncoderPosition = getAbsPosition();

        Logger.recordOutput("elevator/motionMagicEnabled", wristMotor.getMotionMagicIsRunning().getValue());
    }

    @Override
    public void setPercent(double percent) {
        wristMotor.set(percent);
    }

    //    @Override
    //    public void setSetpoint(double position, double velocity, double acceleration) {
    //        throw new UnsupportedOperationException("Not supported yet.");
    //        //            wristMotor.setControl(motionMagic.withPosition(extension));
    //    }

    @Override
    public void setMotionMagic(double position) {
        Logger.recordOutput("elevator/lastMotionMagic", position);
        wristMotor.setControl(motionMagic.withPosition(position / (2 * Math.PI)));
    }

    @Override
    public void setVolts(double voltage) {
        wristMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        wristMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double getAbsPosition() {
        return MathUtil.angleModulus(
                (2 * Math.PI) * absoluteEncoder.getAbsPosition() + Math.toRadians(90));
    }
}
