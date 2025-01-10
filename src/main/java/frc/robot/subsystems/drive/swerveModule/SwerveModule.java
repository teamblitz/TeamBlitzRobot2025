/* Big thanks to Team 364 for the base code. */

package frc.robot.subsystems.drive.swerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIO;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIO;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOHelium;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    public final int moduleNumber;
    private Rotation2d lastAngle;

    private final AngleMotorIO angleMotor;
    private final DriveMotorIO driveMotor;
    private final EncoderIO absoluteEncoder;

    private final AngleMotorInputsAutoLogged angleMotorInputs = new AngleMotorInputsAutoLogged();
    private final DriveMotorInputsAutoLogged driveMotorInputs = new DriveMotorInputsAutoLogged();
    private final EncoderIOInputsAutoLogged encoderInputs = new EncoderIOInputsAutoLogged();

    private final SimpleMotorFeedforward driveFeedforward =
            new SimpleMotorFeedforward(
                    Constants.Drive.DRIVE_KS, Constants.Drive.DRIVE_KV, Constants.Drive.DRIVE_KA);
    private final String logKey;

    public SwerveModule(
            int moduleNumber,
            AngleMotorIO angleMotor,
            DriveMotorIO driveMotor,
            EncoderIO absoluteEncoder) {
        this.moduleNumber = moduleNumber;
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.absoluteEncoder = absoluteEncoder;

        // hehe
        absoluteEncoder.updateInputs(encoderInputs);
        resetToAbs();

        lastAngle = getAngle();

        logKey = "drive/mod" + moduleNumber;
    }

    public void configAnglePid(double p, double i, double d) {
        angleMotor.configurePID(p, i, d);
    }

    public void configDrivePid(double p, double i, double d) {
        driveMotor.configurePID(p, i, d);
    }

    public void periodic() {
        angleMotor.updateInputs(angleMotorInputs);
        driveMotor.updateInputs(driveMotorInputs);
        absoluteEncoder.updateInputs(encoderInputs);

        Logger.processInputs(logKey + "/angle", angleMotorInputs);
        Logger.processInputs(logKey + "/drive", driveMotorInputs);
        Logger.processInputs(logKey + "/absEncoder", encoderInputs);
    }

    public void setDesiredState(
            SwerveModuleState desiredState, boolean isOpenLoop, boolean tuning, boolean parking) {

        desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle);

        setAngle(desiredState, tuning, parking);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Drive.MAX_SPEED;
            driveMotor.setDrivePercent(percentOutput);
            Logger.recordOutput(logKey + "/drivePercent", percentOutput);
        } else {
            double speed = MathUtil.applyDeadband(desiredState.speedMetersPerSecond, .01);
            Logger.recordOutput(logKey + "/speedSetpoint", speed);
            driveMotor.setSetpoint(speed, driveFeedforward.calculate(speed));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean tuning, boolean parking) {
        Rotation2d angle =
                (!(tuning || parking)
                                && Math.abs(desiredState.speedMetersPerSecond)
                                        <= (Constants.Drive.MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle; // Prevent rotating module if speed is less than 1%.
        angleMotor.setSetpoint(angle.getDegrees());
        Logger.recordOutput(logKey + "/angleSetpoint", angle.getDegrees());
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleMotorInputs.rotation);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(encoderInputs.position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotorInputs.velocity, getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotorInputs.position, getAngle());
    }

    public void setBrakeMode(boolean enabled) {
        driveMotor.setBrakeMode(enabled);
    }

    public void zeroAbsEncoders() {
        ((EncoderIOHelium) absoluteEncoder).zeroEncoder();
    }

    public void resetToAbs() {
        angleMotor.seedPosition(encoderInputs.position);
    }

    public double getVelocity() {
        return driveMotorInputs.velocity;
    }

    public double getVoltsDrive() {
        return driveMotorInputs.volts;
    }

    public double getPositionDrive() {
        return driveMotorInputs.position;
    }
}
