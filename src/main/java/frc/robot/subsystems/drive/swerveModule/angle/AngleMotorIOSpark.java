package frc.robot.subsystems.drive.swerveModule.angle;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class AngleMotorIOSpark implements AngleMotorIO {
    private final Rotation2d angleOffset;

    private final SparkMax motor;

    private final RelativeEncoder encoder;

    private final SparkClosedLoopController pidController;

    public AngleMotorIOSpark(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Motor */
        motor = new SparkMax(moduleConstants.angleMotorID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();
        configAngleMotor();
    }

    @Override
    public void updateInputs(AngleMotorIO.AngleMotorInputs inputs) {
        inputs.angularVelocity = encoder.getVelocity();
        inputs.rotation = encoder.getPosition();
    }

    @Override
    public void setSetpoint(double setpoint) {
        pidController.setReference(setpoint, SparkMax.ControlType.kPosition);
    }

    @Override
    public void configurePID(double p, double i, double d) {
        motor.configure(
                new SparkMaxConfig().apply(new ClosedLoopConfig().pid(p, i, d)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void seedPosition(double position) {
        encoder.setPosition(position - angleOffset.getDegrees());
    }

    private void configAngleMotor() {

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(Constants.Drive.ANGLE_SMART_CURRENT_LIMIT)
                .secondaryCurrentLimit(Constants.Drive.ANGLE_SECONDARY_CURRENT_LIMIT)
                .inverted(Constants.Drive.ANGLE_MOTOR_INVERT)
                .idleMode(Constants.Drive.ANGLE_BRAKE_MODE ? IdleMode.kBrake : IdleMode.kCoast);

        config.encoder.positionConversionFactor(
                ((1 / Constants.Drive.ANGLE_GEAR_RATIO) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360)); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        configurePID(Constants.Drive.ANGLE_KP, Constants.Drive.ANGLE_KI, Constants.Drive.ANGLE_KD);
    }
}
