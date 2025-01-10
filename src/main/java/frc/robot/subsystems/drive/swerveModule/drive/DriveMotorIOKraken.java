package frc.robot.subsystems.drive.swerveModule.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class DriveMotorIOKraken implements DriveMotorIO {
    private final TalonFX motor;

    private final VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private final DutyCycleOut openLoopDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
    private boolean brakeEnabled = false;

    public DriveMotorIOKraken(SwerveModuleConstants moduleConstants) {

        /* Drive motor */
        motor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
    }

    @Override
    public void updateInputs(DriveMotorIO.DriveMotorInputs inputs) {
        inputs.position = motor.getPosition().getValueAsDouble();
        inputs.velocity = motor.getVelocity().getValueAsDouble();
        inputs.volts = motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setDrivePercent(double percent) {
        motor.setControl(openLoopDutyCycle.withOutput(percent));
    }

    @Override
    public void setSetpoint(double setpoint, double ffVolts) {
        motor.setControl(
                closedLoopVelocity.withVelocity(setpoint).withSlot(0).withFeedForward(ffVolts));
    }

    @Override
    public void configurePID(double p, double i, double d) {
        motor.getConfigurator().apply(new Slot0Configs().withKP(p).withKI(i).withKD(d));
    }

    private void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.withInverted(
                        Constants.Drive.DRIVE_MOTOR_INVERT
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(
                        Constants.Drive.DRIVE_BRAKE_MODE
                                ? NeutralModeValue.Brake
                                : NeutralModeValue.Coast);

        config.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(Constants.Drive.OPEN_LOOP_RAMP);

        config.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(Constants.Drive.CLOSED_LOOP_RAMP);

        config.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.Drive.CurrentLimits.Kraken.DRIVE_STATOR);

        config.Feedback.withSensorToMechanismRatio(
                Constants.Drive.DRIVE_GEAR_RATIO * (1 / Constants.Drive.WHEEL_CIRCUMFERENCE));

        motor.getConfigurator().apply(config);

        configurePID(Constants.Drive.DRIVE_KP, Constants.Drive.DRIVE_KI, Constants.Drive.DRIVE_KD);

        motor.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100, motor.getVelocity(), motor.getPosition(), motor.getMotorVoltage());
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (brakeEnabled == enabled) return;
        motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        brakeEnabled = enabled;
    }
}
