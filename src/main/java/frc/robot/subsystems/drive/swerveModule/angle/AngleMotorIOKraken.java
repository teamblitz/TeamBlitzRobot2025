package frc.robot.subsystems.drive.swerveModule.angle;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.monitor.HardwareWatchdog;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class AngleMotorIOKraken implements AngleMotorIO {
    private final TalonFX motor;

    private final PositionVoltage closedLoopPosition = new PositionVoltage(0).withEnableFOC(true);

    private final Rotation2d angleOffset;

    public AngleMotorIOKraken(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle motor */
        motor = new TalonFX(moduleConstants.angleMotorID, "drive");
        configAngleMotor();

        HardwareWatchdog.getInstance().registerCTREDevice(motor, this.getClass());
    }

    @Override
    public void updateInputs(AngleMotorIO.AngleMotorInputs inputs) {
        inputs.angularVelocity = motor.getVelocity().getValueAsDouble();
        inputs.rotation = motor.getPosition().getValueAsDouble();
    }

    @Override
    public void seedPosition(double position) {
        motor.setPosition(position - angleOffset.getDegrees());
    }

    @Override
    public void setSetpoint(double setpoint) {
        motor.setControl(closedLoopPosition.withPosition(setpoint));
    }

    @Override
    public void configurePID(double p, double i, double d) {
        motor.getConfigurator().apply(new Slot0Configs().withKP(p).withKI(i).withKD(d));
    }

    private void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withInverted(
                        Constants.Drive.ANGLE_MOTOR_INVERT
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(
                        Constants.Drive.ANGLE_BRAKE_MODE
                                ? NeutralModeValue.Brake
                                : NeutralModeValue.Coast);

        // TODO: CURRENT LIMIT ASAP

        config.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Drive.CurrentLimits.Kraken.DRIVE_STATOR);

        config.Feedback.withSensorToMechanismRatio(Constants.Drive.ANGLE_GEAR_RATIO / 360.0);

        motor.getConfigurator().apply(config);

        configurePID(Constants.Drive.ANGLE_KP, Constants.Drive.ANGLE_KI, Constants.Drive.ANGLE_KD);
    }
}
