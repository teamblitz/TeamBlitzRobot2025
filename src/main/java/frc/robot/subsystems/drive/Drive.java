/* Big thanks to Team 364 for the base swerve code. */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Drive.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.util.*;
import frc.robot.Constants;
import frc.robot.PositionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.control.*;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.range.RangeSensorIO;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.range.RangeSensorIOInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOKraken;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOKraken;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOCanCoder;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOHelium;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends BlitzSubsystem {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] swerveModules;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final RangeSensorIO rangeIO;
    private final RangeSensorIOInputsAutoLogged rangeInputs = new RangeSensorIOInputsAutoLogged();
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");
    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("DriveTuning");

    private final LoggedTunableNumber angleP = new LoggedTunableNumber("drive/angle/kP", ANGLE_KP);
    private final LoggedTunableNumber angleI = new LoggedTunableNumber("drive/angle/kI", ANGLE_KI);
    private final LoggedTunableNumber angleD = new LoggedTunableNumber("drive/angle/kD", ANGLE_KD);

    private final LoggedTunableNumber driveP = new LoggedTunableNumber("drive/drive/kP", ANGLE_KP);
    private final LoggedTunableNumber driveI = new LoggedTunableNumber("drive/drive/kI", ANGLE_KI);
    private final LoggedTunableNumber driveD = new LoggedTunableNumber("drive/drive/kD", ANGLE_KD);

    private double lastTurnCommandSeconds;
    private boolean keepHeadingSetpointSet;

    private final PIDController keepHeadingPid;
    private final ProfiledPIDController rotateToHeadingPid;

    private SysIdRoutine linearRoutine;
    private SysIdRoutine angularRoutine;

    private Rotation2d gyroOffset = new Rotation2d();

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public Drive(
            SwerveModuleConfiguration configuration,
            SwerveModuleConstants flConstants,
            SwerveModuleConstants frConstants,
            SwerveModuleConstants blConstants,
            SwerveModuleConstants brConstants,
            GyroIO gyroIO,
            RangeSensorIO rangeIO) {
        this(
                new SwerveModule(
                        FL,
                        configuration.angle == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new AngleMotorIOKraken(flConstants)
                                : new AngleMotorIOSpark(flConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(flConstants)
                                : new DriveMotorIOSpark(flConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(flConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(flConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        FR,
                        configuration.angle == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new AngleMotorIOKraken(frConstants)
                                : new AngleMotorIOSpark(frConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(frConstants)
                                : new DriveMotorIOSpark(frConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(frConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(frConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BL,
                        configuration.angle == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new AngleMotorIOKraken(blConstants)
                                : new AngleMotorIOSpark(blConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(blConstants)
                                : new DriveMotorIOSpark(blConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(blConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(blConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BR,
                        configuration.angle == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new AngleMotorIOKraken(brConstants)
                                : new AngleMotorIOSpark(brConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(brConstants)
                                : new DriveMotorIOSpark(brConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(brConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(brConstants.cancoderID, CAN_CODER_INVERT)),
                gyroIO,
                rangeIO);
    }

    public Drive(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            GyroIO gyroIO,
            RangeSensorIO rangeIO) {
        super("drive");

        swerveModules =
                new SwerveModule[] { // front left, front right, back left, back right.
                    frontLeft, frontRight, backLeft, backRight
                };

        swerveOdometry = new SwerveDriveOdometry(KINEMATICS, getYaw(), getModulePositions());
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

        this.gyroIO = gyroIO;
        this.rangeIO = rangeIO;

        keepHeadingPid = new PIDController(.15, 0, 0);
        keepHeadingPid.enableContinuousInput(-180, 180);
        keepHeadingPid.setTolerance(2);

        rotateToHeadingPid = new ProfiledPIDController(.1, 0, 0, new Constraints(180, 360));
        rotateToHeadingPid.enableContinuousInput(-180, 180);
        rotateToHeadingPid.setTolerance(2);
        initTelemetry();

        //        zeroGyro();

        new RangeSensorIOFusion();

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.runOnce(() -> keepHeadingSetpointSet = false));

        // Especially on the Helium CanAndMag encoders, sometimes a sensor reading to seed the
        // module isn't available at startup. To fix this we wait 3 seconds and then reseed all
        // modules
        Commands.waitSeconds(3)
                .andThen(
                        Commands.runOnce(
                                        () -> {
                                            for (SwerveModule module : swerveModules) {
                                                module.resetToAbs();
                                            }
                                        })
                                .ignoringDisable(true))
                .schedule();

        // The above wasn't working for some reason on devbot this year, the below code seeds the
        // module positions every
        // 2 seconds until the robot is enabled for the first time.
        //        if (!Constants.compBot()) {
        //            Commands.sequence(
        //                            Commands.waitSeconds(2),
        //                            Commands.runOnce(
        //                                    () -> {
        //                                        for (SwerveModule module : swerveModules) {
        //                                            module.resetToAbs();
        //                                        }
        //                                    }))
        //                    .repeatedly()
        //                    .withName("SWERVE FIX")
        //                    .until(DriverStation::isEnabled)
        //                    .onlyIf(() -> !Constants.compBot())
        //                    .ignoringDisable(true)
        //                    .schedule();
        //        }

        // Creates a SysIdRoutine
        linearRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                Seconds.of(10),
                                Constants.compBot()
                                        ? (state) ->
                                                SignalLogger.writeString(
                                                        "sysid-drive-linear-state",
                                                        state.toString())
                                        : (state) ->
                                                Logger.recordOutput(
                                                        "sysid-angular drive", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> {
                                    var speeds =
                                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                                    new ChassisSpeeds(volts.in(Volt), 0, 0),
                                                    getYaw());

                                    SwerveModuleState[] desiredStates =
                                            KINEMATICS.toSwerveModuleStates(speeds);
                                    for (SwerveModule mod : swerveModules) {
                                        mod.setStatesVoltage(desiredStates[mod.moduleNumber]);
                                    }
                                },
                                null,
                                this));

        angularRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                Seconds.of(10),
                                Constants.compBot()
                                        ? (state) ->
                                                SignalLogger.writeString(
                                                        "sysid-drive-angular-state",
                                                        state.toString())
                                        : (state) ->
                                                Logger.recordOutput(
                                                        "sysid-angular-drive", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> {
                                    SwerveModuleState[] desiredStates =
                                            KINEMATICS.toSwerveModuleStates(
                                                    new ChassisSpeeds(0, 0, volts.in(Volt)));
                                    for (SwerveModule mod : swerveModules) {
                                        mod.setStatesVoltage(desiredStates[mod.moduleNumber]);
                                    }
                                },
                                null,
                                this));

        // https://pathplanner.dev/pplib-swerve-setpoint-generator.html
        setpointGenerator =
                new SwerveSetpointGenerator(
                        PHYSICAL_CONSTANTS, // The robot configuration. This is the same config used
                        // for generating trajectories and running path
                        // following commands.
                        MAX_MODULE_ANGULAR_VELOCITY // The max rotation velocity of a swerve module
                        // in radians per second.
                        );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds =
                getChassisSpeeds(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates =
                getModuleStates(); // Method to get the current swerve module states
        previousSetpoint =
                new SwerveSetpoint(
                        currentSpeeds,
                        currentStates,
                        DriveFeedforwards.zeros(PHYSICAL_CONSTANTS.numModules));

        tuningTab.add(
                linearAysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("DriveLinear Quasistic Forward"));
        tuningTab.add(
                linearAysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("DriveLinear Quasistic Reverse"));

        tuningTab.add(
                linearSysIdDynamic(SysIdRoutine.Direction.kForward)
                        .withName("DriveLinear Dynamic Forward"));
        tuningTab.add(
                linearSysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .withName("DriveLinear Dynamic Reverse"));

        tuningTab.add(
                angularSysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("DriveAngular Quasistic Forward"));
        tuningTab.add(
                angularSysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("DriveAngular Quasistic Reverse"));

        tuningTab.add(
                angularSysIdDynamic(SysIdRoutine.Direction.kForward)
                        .withName("DriveAngular Dynamic Forward"));
        tuningTab.add(
                angularSysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .withName("DriveAngular Dynamic Reverse"));

        for (ScoringPositions.Branch branch : ScoringPositions.Branch.values()) {
            SmartDashboard.putData(
                    branch.name(),
                    driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(branch)));
        }
    }

    public void drive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop,
            boolean maintainHeading) {

        angleDrive(translation, rotation, 0, fieldRelative, isOpenLoop, maintainHeading, false);
    }

    double lastRotationSetpoint;

    public void angleDrive(
            Translation2d translation,
            double rotation,
            double rotationSetpoint,
            boolean fieldRelative,
            boolean isOpenLoop,
            boolean maintainHeading,
            boolean doRotationPid) {
        if (doRotationPid) {
            keepHeadingSetpointSet = false;

            if (lastRotationSetpoint != rotationSetpoint) {
                rotateToHeadingPid.reset(new TrapezoidProfile.State(getYaw().getDegrees(), 0));
                lastRotationSetpoint = rotationSetpoint;
            }

            rotation = rotateToHeadingPid.calculate(getYaw().getDegrees(), rotationSetpoint);
        } else {
            if (rotation != 0) {
                lastTurnCommandSeconds = Timer.getFPGATimestamp();
                keepHeadingSetpointSet = false;
                Logger.recordOutput("Drive/Turning", true);
            }
            if (lastTurnCommandSeconds + .5 <= Timer.getFPGATimestamp()
                    && !keepHeadingSetpointSet) { // If it has been at least .5 seconds.
                keepHeadingPid.setSetpoint(getYaw().getDegrees());
                keepHeadingSetpointSet = true;
                Logger.recordOutput("Drive/Turning", false);
            }

            if (keepHeadingSetpointSet && maintainHeading) {
                rotation = keepHeadingPid.calculate(getYaw().getDegrees());
            }
        }

        Logger.recordOutput("Drive/keepHeadingSetpointSet", keepHeadingSetpointSet);
        Logger.recordOutput("Drive/keepSetpoint", keepHeadingPid.getSetpoint());

        var fieldRelativeSpeeds =
                new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        var flippedFieldSpeeds =
                new ChassisSpeeds(
                        -fieldRelativeSpeeds.vxMetersPerSecond,
                        -fieldRelativeSpeeds.vyMetersPerSecond,
                        rotation);

        var correctFieldSpeeds =
                AllianceFlipUtil.shouldFlip() ? flippedFieldSpeeds : fieldRelativeSpeeds;

        ChassisSpeeds robotRel =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(correctFieldSpeeds, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        drive(robotRel, isOpenLoop);
    }

    public void driveFieldRelative(ChassisSpeeds speeds, boolean openLoop) {
        Logger.recordOutput("drive/requestedFieldSpeeds", speeds);
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw()), openLoop);
    }

    public void drive(ChassisSpeeds speeds, boolean openLoop) {
        Logger.recordOutput("drive/requestedSpeeds", speeds);

        ChassisSpeeds discretizedSpeeds =
                ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SEC);

        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        Logger.recordOutput(logKey + "/drivespeeds", speeds);
        Logger.recordOutput(logKey + "/driveopen", openLoop);
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint =
                setpointGenerator.generateSetpoint(
                        previousSetpoint, // The previous setpoint
                        speeds, // The desired target speeds
                        Constants.LOOP_PERIOD_SEC // The loop time of the robot code, in
                        );

        Logger.recordOutput(logKey + "/setpoint/speeds", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput(logKey + "/setpoint/states", previousSetpoint.moduleStates());
        Logger.recordOutput(logKey + "/setpoint/ff", previousSetpoint.feedforwards());

        setModuleStates(swerveModuleStates, openLoop, false, false);
    }

    /* Used by SwerveControllerCommand in Auto */
    // Use in above method?
    public void setModuleStates(
            SwerveModuleState[] desiredStates, boolean openLoop, boolean tuning, boolean parking) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        Logger.recordOutput("drive/desiredSwerveStates", desiredStates);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop, tuning, parking);
        }
    }

    public Command park() {
        SwerveModuleState[] desiredStates = {
            (new SwerveModuleState(0, Rotation2d.fromDegrees(45))),
            (new SwerveModuleState(0, Rotation2d.fromDegrees(-45))),
            (new SwerveModuleState(0, Rotation2d.fromDegrees(-45))),
            (new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        };

        return runOnce(() -> setModuleStates(desiredStates, true, false, true))
                .withName(logKey + "/park");
    }

    public void setBrakeMode(boolean enabled) {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setBrakeMode(enabled);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);

        if (Robot.isSimulation()) {
            setGyro(pose.getRotation().getDegrees());
        }
    }

    public void addVisionMeasurement(
            Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {

        poseEstimator.addVisionMeasurement(
                pose,
                timestamp,
                visionMeasurementStdDevs); // Maybe base std devs off of camera stuff, .7m seams
        // high as an std
        // Standard deviations, basically vision

    }

    public void zeroGyro() {
        setGyro(0);
    }

    public void setGyro(double degrees) {
        gyroOffset = Rotation2d.fromDegrees(degrees).minus(Rotation2d.fromDegrees(gyroInputs.yaw));
        keepHeadingSetpointSet = false;
        lastTurnCommandSeconds = Timer.getFPGATimestamp();
    }

    @AutoLogOutput(key = "gyro/getyaw")
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyroInputs.yaw).plus(gyroOffset);
    }

    public double getPitch() {
        return gyroInputs.pitch;
    }

    public double getRoll() {
        return gyroInputs.roll;
    }

    public double getRange() {
        return rangeInputs.range;
    }

    public Command driveSpeedTestCommand(double speed, double duration) {
        SlewRateLimiter filter = new SlewRateLimiter(1);
        return Commands.run(
                        () ->
                                drive(
                                        new Translation2d(filter.calculate(speed), 0),
                                        0,
                                        false,
                                        false,
                                        false))
                .withTimeout(duration)
                .andThen(
                        Commands.run(
                                () ->
                                        drive(
                                                new Translation2d(filter.calculate(0), 0),
                                                0,
                                                false,
                                                false,
                                                false)));
    }

    /**
     * Chase a field relative vector
     *
     * @param vector robot relative unit vector to move in the direction of
     * @param angle angle error
     * @param velocity goal velocity m/s
     * @param acceleration max acceleration m/s^2
     * @return Chase Vector Command.
     */
    public Command chaseVector(
            Supplier<Translation2d> vector,
            DoubleSupplier angle,
            double velocity,
            double acceleration) {
        SlewRateLimiter xLimiter = new SlewRateLimiter(acceleration);
        SlewRateLimiter yLimiter = new SlewRateLimiter(acceleration);

        return Commands.runOnce(
                        () -> {
                            xLimiter.reset(getFieldRelativeSpeeds().vxMetersPerSecond);
                            yLimiter.reset(getFieldRelativeSpeeds().vyMetersPerSecond);
                        })
                .andThen(
                        run(
                                () -> {
                                    Translation2d unitVec =
                                            vector.get().div(vector.get().getNorm());
                                    Translation2d goalSpeeds = unitVec.times(velocity);

                                    angleDrive(
                                            new Translation2d(
                                                    xLimiter.calculate(goalSpeeds.getX()),
                                                    yLimiter.calculate(goalSpeeds.getY())),
                                            angle.getAsDouble() * .0 * MAX_ANGULAR_VELOCITY,
                                            0,
                                            true,
                                            true,
                                            true,
                                            false);
                                }))
                .withName(logKey + "/chaseVector");
    }

    public Command zeroAbsEncoders() {
        return runOnce(
                        () -> {
                            swerveModules[0].zeroAbsEncoders();
                            swerveModules[1].zeroAbsEncoders();
                            swerveModules[2].zeroAbsEncoders();
                            swerveModules[3].zeroAbsEncoders();
                        })
                .ignoringDisable(true)
                .withName(logKey + "/zeroAbsEncoders");
    }

    public Command linearAysIdQuasistatic(SysIdRoutine.Direction direction) {

        Rotation2d rot = Rotation2d.kZero;

        return Commands.sequence(
                        run(() ->
                                        setModuleStates(
                                                new SwerveModuleState[] {
                                                    new SwerveModuleState(0, rot),
                                                    new SwerveModuleState(0, rot),
                                                    new SwerveModuleState(0, rot),
                                                    new SwerveModuleState(0, rot)
                                                },
                                                true,
                                                true,
                                                true))
                                .withTimeout(1),
                        linearRoutine.quasistatic(direction))
                .withName(
                        logKey
                                + "/linearQuasistatic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command linearSysIdDynamic(SysIdRoutine.Direction direction) {

        Rotation2d rot = Rotation2d.kZero;

        return Commands.sequence(
                        run(() ->
                                        setModuleStates(
                                                new SwerveModuleState[] {
                                                    new SwerveModuleState(0, rot),
                                                    new SwerveModuleState(0, rot),
                                                    new SwerveModuleState(0, rot),
                                                    new SwerveModuleState(0, rot)
                                                },
                                                true,
                                                true,
                                                true))
                                .withTimeout(1),
                        linearRoutine.dynamic(direction))
                .withName(
                        logKey
                                + "/linearDynamic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command angularSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return linearRoutine
                .quasistatic(direction)
                .withName(
                        logKey
                                + "/angularQuasistatic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command angularSysIdDynamic(SysIdRoutine.Direction direction) {
        return linearRoutine
                .quasistatic(direction)
                .withName(
                        logKey
                                + "/angularDynamic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    @Override
    public void periodic() {
        super.periodic();

        for (SwerveModule mod : swerveModules) {
            mod.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        rangeIO.updateInputs(rangeInputs);
        Logger.processInputs("gyro", gyroInputs);
        Logger.processInputs("drive/range", rangeInputs);

        swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.update(getYaw(), getModulePositions());

        Logger.recordOutput(logKey + "/odometry", swerveOdometry.getPoseMeters());
        Logger.recordOutput(logKey + "/poseEstimator", poseEstimator.getEstimatedPosition());

        Logger.recordOutput(logKey + "/modules", getModuleStates());

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (pid) -> {
                    for (SwerveModule module : swerveModules)
                        module.configAnglePid(pid[0], pid[1], pid[2]);
                },
                angleP,
                angleI,
                angleD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (pid) -> {
                    for (SwerveModule module : swerveModules)
                        module.configDrivePid(pid[0], pid[1], pid[2]);
                },
                driveP,
                driveI,
                driveD);
    }

    public void initTelemetry() {
        //        tuningTab.add("KeepHeadingPid", keepHeadingPid);
        // tuningTab.add("Tuning Command", new SwerveTuning(this));
    }

    public Optional<Pose2d> samplePreviousPose(double timestamp) {
        return poseEstimator.sampleAt(timestamp);
    }

    private final PIDController xController = new PIDController(14, 0, 0);
    private final PIDController yController = new PIDController(14, 0, 0);
    private final PIDController choreoThetaController = new PIDController(3, 0, 0);

    public void followTrajectory(SwerveSample sample) {
        Logger.recordOutput("drive/choreoTrajectory", sample);
        choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        sample.vx
                                + xController.calculate(
                                        poseEstimator.getEstimatedPosition().getX(), sample.x),
                        sample.vy
                                + yController.calculate(
                                        poseEstimator.getEstimatedPosition().getY(), sample.y),
                        sample.omega
                                + choreoThetaController.calculate(
                                        poseEstimator
                                                .getEstimatedPosition()
                                                .getRotation()
                                                .getRadians(),
                                        sample.heading));

        driveFieldRelative(speeds, false);
    }

    /* Drive to Pose */

    /*
     * The below code is based on 6995 Nomad's drive to pose code.
     * https://github.com/frc6995/Robot-2025/blob/ec7eccecc7d6793f9a4bf5057c5cf5b1bfb0f65d/src/main/java/frc/robot/subsystems/DriveBaseS.java#L524
     */

    /* DRIVE TO POSE using Trapezoid Profiles */
    private TrapezoidProfile.Constraints driveToPoseConstraints = new Constraints(1, 1);
    private TrapezoidProfile.Constraints driveToPoseRotationConstraints = new Constraints(3, 6);
    private TrapezoidProfile driveToPoseProfile = new TrapezoidProfile(driveToPoseConstraints);
    private TrapezoidProfile driveToPoseRotationProfile =
            new TrapezoidProfile(driveToPoseRotationConstraints);

    // For modifying goals from within a lambda

    private TrapezoidProfile.State driveToPoseGoal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State driveToPoseRotationGoal = new TrapezoidProfile.State(0, 0);

    Capture<Pose2d> initial = new Capture<Pose2d>(new Pose2d());
    // The goal (populated from poseSupplier at command start)
    Capture<Pose2d> goal = new Capture<Pose2d>(new Pose2d());
    // Distance start-end in meters
    Capture<Double> distance = new Capture<Double>(1.0);
    // Unit vector start->end
    Capture<Translation2d> normDirStartToEnd = new Capture<>(Translation2d.kZero);
    Capture<Vector<N2>> directionGoalToBot = new Capture<>(VecBuilder.fill(0, 0));
    TrapezoidProfile.State translationState = new State(0, 0);
    TrapezoidProfile.State rotationState = new State(0, 0);

    // Threshold for "close enough" to avoid microadjustments
    public final Trigger atDriveToPosePose =
            atPose(() -> goal.inner, Units.inchesToMeters(0.5), Units.degreesToRadians(1));

    /**
     * <B>IMPORTANT, While this takes a pose supplier, this is mostly for convince. and <U>once the
     * command has started, the command will sample the value of the supplier and drive there.</U>
     * It will NOT follow the pose</B>
     *
     * <p>Drives to a pose with motion profiles on translation and rotation. The translation profile
     * starts at dist(start,end) and drives toward 0. This state is then interpolated between poses.
     *
     * <p>The rotation profile starts at initial.heading and ends at goal.heading, just like a
     * profiled continuous heading controller.
     */
    public Command driveToPose(Supplier<Pose2d> poseSupplier) {
        Command command =
                runOnce(
                                () -> {
                                    var getTargetTime = Timer.getFPGATimestamp();

                                    initial.inner = getPose();
                                    goal.inner = poseSupplier.get();

                                    // initial position: distance from end
                                    // initial velocity: component of velocity away from end, so
                                    // approaching is a negative number
                                    var goalToBot = initial.inner.minus(goal.inner);
                                    var directionGoalToBot =
                                            goalToBot.getTranslation().toVector().unit();

                                    this.directionGoalToBot.inner = directionGoalToBot;
                                    // TODO: The bellow being commented out means that there is no
                                    // velocity feedforward
                                    //
                                    // this.normDirStartToEnd.inner = new
                                    // Translation2d(directionGoalToBot);

                                    distance.inner = goalToBot.getTranslation().getNorm();

                                    // Position goes from our distance to zero as we approach
                                    translationState.position = distance.inner;

                                    var speeds = getFieldRelativeSpeeds();

                                    // A negative velocity means we are approaching 0, the goal is 0
                                    translationState.velocity =
                                            MathUtil.clamp(
                                                    VecBuilder.fill(
                                                                    speeds.vxMetersPerSecond,
                                                                    speeds.vyMetersPerSecond)
                                                            .dot(directionGoalToBot),
                                                    -driveToPoseConstraints.maxVelocity,
                                                    0);

                                    Logger.recordOutput(
                                            "drive/driveToPose/unclampedInitialVelocity",
                                            VecBuilder.fill(
                                                            speeds.vxMetersPerSecond,
                                                            speeds.vyMetersPerSecond)
                                                    .dot(directionGoalToBot));

                                    // Initial state of rotation
                                    driveToPoseRotationGoal.position =
                                            goal.inner.getRotation().getRadians();

                                    rotationState.position =
                                            initial.inner.getRotation().getRadians();
                                    rotationState.velocity = speeds.omegaRadiansPerSecond;

                                    Logger.recordOutput("drive/driveToPose/initial", initial.inner);
                                    Logger.recordOutput("drive/driveToPose/goal", goal.inner);
                                    Logger.recordOutput(
                                            "drive/driveToPose/initialDistance",
                                            translationState.position);
                                    Logger.recordOutput(
                                            "drive/driveToPose/initialVelocity",
                                            translationState.velocity);
                                })
                        .andThen(
                                run(
                                        () -> {
                                            var setpoint =
                                                    driveToPoseProfile.calculate(
                                                            Constants.LOOP_PERIOD_SEC,
                                                            translationState,
                                                            driveToPoseGoal);
                                            translationState.position = setpoint.position;
                                            translationState.velocity = setpoint.velocity;

                                            Logger.recordOutput(
                                                    "drive/driveToPose/distance",
                                                    translationState.position);
                                            Logger.recordOutput(
                                                    "drive/driveToPose/velocity",
                                                    translationState.velocity);

                                            // I am trusting them here

                                            // Rotation continuous input
                                            // Get error which is the smallest distance between goal
                                            // and measurement
                                            double errorBound = Math.PI;
                                            var measurement = getYaw().getRadians();
                                            double goalMinDistance =
                                                    MathUtil.inputModulus(
                                                            driveToPoseRotationGoal.position
                                                                    - measurement,
                                                            -errorBound,
                                                            errorBound);
                                            double setpointMinDistance =
                                                    MathUtil.inputModulus(
                                                            rotationState.position - measurement,
                                                            -errorBound,
                                                            errorBound);

                                            // Recompute the profile goal with the smallest error,
                                            // thus giving the shortest path. The goal
                                            // may be outside the input range after this operation,
                                            // but that's OK because the controller
                                            // will still go there and report an error of zero. In
                                            // other words, the setpoint only needs to
                                            // be offset from the measurement by the input range
                                            // modulus; they don't need to be equal.
                                            driveToPoseRotationGoal.position =
                                                    goalMinDistance + measurement;
                                            rotationState.position =
                                                    setpointMinDistance + measurement;

                                            var rotSetpoint =
                                                    driveToPoseRotationProfile.calculate(
                                                            0.02,
                                                            rotationState,
                                                            driveToPoseRotationGoal);
                                            rotationState.position = rotSetpoint.position;
                                            rotationState.velocity = rotSetpoint.velocity;

                                            var startPose = initial.inner;

                                            var interpTrans =
                                                    goal.inner
                                                            .getTranslation()
                                                            .interpolate(
                                                                    startPose.getTranslation(),
                                                                    setpoint.position
                                                                            / distance.inner);

                                            if (atDriveToPosePose.getAsBoolean()) {
                                                this.drive(new ChassisSpeeds(), true);
                                            } else {
                                                followTrajectory(
                                                        DriveUtil.sample(
                                                                interpTrans,
                                                                new Rotation2d(
                                                                        rotationState.position),
                                                                normDirStartToEnd.inner.getX()
                                                                        * setpoint.velocity,
                                                                normDirStartToEnd.inner.getY()
                                                                        * setpoint.velocity,
                                                                rotationState.velocity));
                                            }
                                            keepHeadingSetpointSet = false;
                                        }))
                        .until(atDriveToPosePose.debounce(.1))
                        .finallyDo(() -> drive(new ChassisSpeeds(), true));

        return command.withName("drive/driveToPoseCommand");
    }

    public Command driveToPose(Pose2d pose) {
        return driveToPose(() -> pose);
    }

    public double toleranceMeters = Units.inchesToMeters(0.5);
    public double toleranceRadians = Units.degreesToRadians(1);

    private boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
        if (Math.abs(toleranceRadians) > Math.PI) {
            return true;
        }
        double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
        // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi],
        // as pre-checked
        // above.
        return dot > Math.cos(toleranceRadians);
    }

    public Trigger atPose(
            Supplier<Pose2d> poseSup, double toleranceMeters, double toleranceRadians) {
        return new Trigger(
                () -> {
                    Pose2d pose = poseSup.get();
                    Pose2d currentPose = getPose();
                    boolean transValid =
                            currentPose.getTranslation().getDistance(pose.getTranslation())
                                    < toleranceMeters;
                    boolean rotValid =
                            withinTolerance(
                                    currentPose.getRotation(),
                                    pose.getRotation(),
                                    toleranceRadians);
                    return transValid && rotValid;
                });
    }

    public Trigger atPose(Supplier<Pose2d> poseSup) {
        return atPose(poseSup, toleranceMeters, toleranceRadians);
    }

    public Trigger atPose(Optional<Pose2d> poseOpt) {
        return poseOpt.map(this::atPose).orElse(new Trigger(() -> false));
    }

    public Trigger atPose(Pose2d pose) {
        return atPose(() -> pose);
    }
}
