/* Big thanks to Team 364 for the base swerve code. */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Drive.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.MutableReference;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.control.*;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.range.RangeSensorIO;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.range.RangeSensorIOInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOKraken;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOCanCoder;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOHelium;
import frc.robot.subsystems.vision.notes.NoteVisionIO;
import frc.robot.subsystems.vision.notes.NoteVisionInputsAutoLogged;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Here we can probably do some cleanup, main thing we can probably do here is separate
 * telemetry/hardware io. Also, we need a better way to do dynamic pid loop tuning.
 */
public class Drive extends BlitzSubsystem {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final TimeInterpolatableBuffer<Pose2d> odoBuffer =
            TimeInterpolatableBuffer.createBuffer(1.5);

    private final SwerveModule[] swerveModules;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final RangeSensorIO rangeIO;
    private final RangeSensorIOInputsAutoLogged rangeInputs = new RangeSensorIOInputsAutoLogged();
    private final NoteVisionIO noteVisionIO;
    private final NoteVisionInputsAutoLogged noteVisionInputs = new NoteVisionInputsAutoLogged();
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

    private SysIdRoutine routine;

    private double lastVisionTimeStamp;

    private Rotation2d gyroOffset = new Rotation2d();
    private final NetworkTableEntry limelightPose =
            LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_wpiblue");

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    // Control/Goals Based control
    private final Subsystem velocityControlMutex = new Subsystem() {};
    private final Subsystem velocityFilteringMutex = new Subsystem() {};
    private final Subsystem headingControlMutex = new Subsystem() {};

    private ChassisSpeedController velocityController = null;
    private ChassisSpeedFilter velocityFilter = null;
    private HeadingController headingController = null;

    NetworkTableEntry intakeTx =
            LimelightHelpers.getLimelightNTTableEntry("limelight-intake", "tx");
    NetworkTableEntry intakeTv =
            LimelightHelpers.getLimelightNTTableEntry("limelight-intake", "tv");

    Debouncer tvBouncer = new Debouncer(2. / 30., Debouncer.DebounceType.kBoth);
    MutableReference<Double> txCache = new MutableReference<>(0.);
    MutableReference<Boolean> tvCache = new MutableReference<>(false);

    public final AmpAssistFilter ampAssistFilter = new AmpAssistFilter(this);
    public final NoteAssistFilter noteAssistFilter = new NoteAssistFilter(this, noteVisionInputs);

    public Command setControl(ChassisSpeedController velocityController) {
        return Commands.startEnd(
                () -> this.velocityController = velocityController,
                () -> this.velocityController = null,
                velocityControlMutex);
    }

    public Command setHeadingControl(HeadingController headingController) {
        return Commands.startEnd(
                () -> this.headingController = headingController,
                () -> this.headingController = null,
                headingControlMutex);
    }

    public Command useVelocityFilter(ChassisSpeedFilter velocityFilter) {
        return Commands.startEnd(
                () -> {
                    this.velocityFilter = velocityFilter;
                    this.velocityFilter.reset();
                },
                () -> this.velocityFilter = null,
                velocityFilteringMutex);
    }

    public Drive(
            SwerveModuleConfiguration configuration,
            SwerveModuleConstants flConstants,
            SwerveModuleConstants frConstants,
            SwerveModuleConstants blConstants,
            SwerveModuleConstants brConstants,
            GyroIO gyroIO,
            RangeSensorIO rangeIO,
            NoteVisionIO noteVisionIO) {
        this(
                new SwerveModule(
                        FL,
                        new AngleMotorIOSpark(flConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(flConstants)
                                : new DriveMotorIOSpark(flConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(flConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(flConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        FR,
                        new AngleMotorIOSpark(frConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(frConstants)
                                : new DriveMotorIOSpark(frConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(frConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(frConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BL,
                        new AngleMotorIOSpark(blConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(blConstants)
                                : new DriveMotorIOSpark(blConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(blConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(blConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BR,
                        new AngleMotorIOSpark(brConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(brConstants)
                                : new DriveMotorIOSpark(brConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(brConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(brConstants.cancoderID, CAN_CODER_INVERT)),
                gyroIO,
                rangeIO,
                noteVisionIO);
    }

    public Drive(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            GyroIO gyroIO,
            RangeSensorIO rangeIO,
            NoteVisionIO noteVisionIO) {
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
        this.noteVisionIO = noteVisionIO;

        keepHeadingPid = new PIDController(.15, 0, 0);
        keepHeadingPid.enableContinuousInput(-180, 180);
        keepHeadingPid.setTolerance(2);

        rotateToHeadingPid = new ProfiledPIDController(.1, 0, 0, new Constraints(180, 360));
        rotateToHeadingPid.enableContinuousInput(-180, 180);
        keepHeadingPid.setTolerance(2);
        initTelemetry();

        zeroGyro();

        new RangeSensorIOFusion();

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.runOnce(() -> keepHeadingSetpointSet = false));

        // Most critical 6 lines of the robot, don't delete, without these it doesn't completely
        // work
        // for some reason
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

        // Creates a SysIdRoutine
        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                Seconds.of(5),
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> {
                                    drive(
                                            new Translation2d(volts.in(Volts) / 12.0, 0)
                                                    .times(Constants.Drive.MAX_SPEED),
                                            0,
                                            false,
                                            true,
                                            true);
                                },
                                log -> {
                                    log.motor("drive")
                                            .voltage(Volts.of(swerveModules[0].getVoltsDrive()))
                                            .linearVelocity(
                                                    MetersPerSecond.of(
                                                            swerveModules[0].getVelocity()))
                                            .linearPosition(
                                                    Meters.of(swerveModules[0].getPositionDrive()));
                                },
                                this));

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                () -> KINEMATICS.toChassisSpeeds(getModuleStates()),
                (speeds, feedforwards) -> drive(speeds, false),
                new PPHolonomicDriveController(
                        AutoConstants.TRANSLATION_PID, AutoConstants.ROTATION_PID),
                PHYSICAL_CONSTANTS,
                () ->
                        DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance()
                                        .get()
                                        .equals(DriverStation.Alliance.Red),
                this);

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
            if (rotation != 0 || velocityFilter == noteAssistFilter) {
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

            if (keepHeadingSetpointSet && maintainHeading && velocityFilter != noteAssistFilter) {
                rotation = keepHeadingPid.calculate(getYaw().getDegrees());
            }
        }

        Logger.recordOutput("Drive/keepHeadingSetpointSet", keepHeadingSetpointSet);
        Logger.recordOutput("Drive/keepSetpoint", keepHeadingPid.getSetpoint());

        ChassisSpeeds robotRel =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        drive(
                velocityFilter != null ? velocityFilter.filterSpeeds(robotRel, false) : robotRel,
                isOpenLoop);
    }

    public void drive(ChassisSpeeds speeds, boolean openLoop) {
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint =
                setpointGenerator.generateSetpoint(
                        previousSetpoint, // The previous setpoint
                        speeds, // The desired target speeds
                        Constants.LOOP_PERIOD_SEC // The loop time of the robot code, in seconds
                        );

        setModuleStates(previousSetpoint.moduleStates(), openLoop, false, false);
    }

    /* Used by SwerveControllerCommand in Auto */
    // Use in above method?
    public void setModuleStates(
            SwerveModuleState[] desiredStates, boolean openLoop, boolean tuning, boolean parking) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

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
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getLimelightPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
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

    // Something something super class???
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction)
                .withName(
                        logKey
                                + "/quasistatic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction)
                .withName(
                        logKey
                                + "/dynamic"
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
        noteVisionIO.updateInputs(noteVisionInputs);
        Logger.processInputs("gyro", gyroInputs);
        Logger.processInputs("drive/range", rangeInputs);
        Logger.processInputs("vision/notes", noteVisionInputs);

        swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.update(getYaw(), getModulePositions());
        odoBuffer.addSample(Timer.getFPGATimestamp(), swerveOdometry.getPoseMeters());

        ///////////////////////////////////////////////////////////////

        // TODO: Whenever this PR gets merged, replace this with it
        // https://github.com/wpilibsuite/allwpilib/pull/6426
        //        Optional<Pose2d> poseTest =
        //                ReflectionHell.samplePoseEstimator(poseEstimator, Timer.getFPGATimestamp()
        // - 1);
        //
        //        poseTest.ifPresent(pose2d -> Logger.recordOutput(logKey + "/poseBufferTest",
        // pose2d));
        //        Logger.recordOutput(logKey + "/poseBufferTestGood", poseTest.isPresent());

        ///////////////////////////////////////////////////////////////

        /* Vision stuff no touchy*/

        LimelightHelpers.PoseEstimate limelightMeasurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (limelightMeasurement != null) {
            if ((limelightMeasurement.tagCount >= 1)
                    && limelightMeasurement.timestampSeconds > lastVisionTimeStamp) {
                poseEstimator.setVisionMeasurementStdDevs(
                        VecBuilder.fill(
                                .7, .7,
                                9999999)); // Standard deviations, basically vision measurements
                // very up
                // to .7m, and just don't trust the vision angle at all (not how std devs work noah)
                poseEstimator.addVisionMeasurement(
                        limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
            }

            lastVisionTimeStamp = limelightMeasurement.timestampSeconds;
        }

        Logger.recordOutput(logKey + "/vision/timestampSeconds", lastVisionTimeStamp);

        Logger.recordOutput(logKey + "/Odometry", swerveOdometry.getPoseMeters());
        Logger.recordOutput(logKey + "/Vision+Odometry", poseEstimator.getEstimatedPosition());
        Logger.recordOutput(logKey + "/Vision", getLimelightPose());
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

        //                noteAssistFilter.apply(new ChassisSpeeds());
        //                noteAssistFilter.apply(new ChassisSpeeds());
    }

    public void initTelemetry() {
        tuningTab.add("KeepHeadingPid", keepHeadingPid);
        // tuningTab.add("Tuning Command", new SwerveTuning(this));
    }

    public Optional<Pose2d> samplePreviousPose(double timestamp) {
        return odoBuffer.getSample(timestamp);
    }
}
