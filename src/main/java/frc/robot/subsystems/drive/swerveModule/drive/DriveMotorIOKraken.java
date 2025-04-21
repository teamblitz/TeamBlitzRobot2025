package frc.robot.subsystems.drive.swerveModule.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.monitor.HardwareWatchdog;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class DriveMotorIOKraken implements DriveMotorIO {
    private final TalonFX motor;

    private final VelocityVoltage closedLoopVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private final DutyCycleOut openLoopDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut openLoopVoltage = new VoltageOut(0).withEnableFOC(true);
    private boolean brakeEnabled = false;

    public DriveMotorIOKraken(SwerveModuleConstants moduleConstants) {

        /* Drive motor */
        motor = new TalonFX(moduleConstants.driveMotorID, "drive");
        configDriveMotor();

        HardwareWatchdog.getInstance().registerCTREDevice(motor, this.getClass());
    }

    @Override
    public void updateInputs(DriveMotorIO.DriveMotorInputs inputs) {
        inputs.position =
                motor.getPosition().getValueAsDouble() * Constants.Drive.WHEEL_CIRCUMFERENCE;
        inputs.velocity =
                motor.getVelocity().getValueAsDouble() * Constants.Drive.WHEEL_CIRCUMFERENCE;
        inputs.volts = motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setDrivePercent(double percent) {
        motor.setControl(openLoopDutyCycle.withOutput(percent));
    }

    @Override
    public void setDriveVolts(double volts) {
        motor.setControl(openLoopVoltage.withOutput(volts));
    }

    double lastVelocity = 0;

    @Override
    public void setSetpoint(double setpoint, double ffVolts) {
        double accel = (setpoint - lastVelocity) / Constants.LOOP_PERIOD_SEC;
        lastVelocity = setpoint;
        Logger.recordOutput("drive/driveIOKraken/velocitySetpointMPS", setpoint);
        Logger.recordOutput(
                "drive/driveIOKraken/velocitySetpointRPS",
                setpoint / Constants.Drive.WHEEL_CIRCUMFERENCE);
        motor.setControl(
                closedLoopVelocity
                        .withVelocity(setpoint / Constants.Drive.WHEEL_CIRCUMFERENCE)
                        .withAcceleration(accel)
                        .withSlot(0));
    }

    @Override
    public void configurePID(double p, double i, double d) {
        //        motor.getConfigurator().apply(new Slot0Configs().withKP(p).withKI(i).withKD(d));
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

        config.Feedback.withSensorToMechanismRatio(Constants.Drive.DRIVE_GEAR_RATIO);

        config.Slot0.withKP(Constants.Drive.DRIVE_KP)
                .withKD(Constants.Drive.DRIVE_KD)
                .withKS(Constants.Drive.DRIVE_KS)
                .withKV(Constants.Drive.DRIVE_KV)
                .withKA(Constants.Drive.DRIVE_KA);

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
