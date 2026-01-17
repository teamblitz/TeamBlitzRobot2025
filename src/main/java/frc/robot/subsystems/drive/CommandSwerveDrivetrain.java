package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.util.Capture;
import frc.lib.util.DriveUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveDrivetrainConstants drivetrainConstants;

    private final LoggedTunableNumber driveKP =
            new LoggedTunableNumber("drive/driveKP", TunerConstants.driveGains.kP);
    private final LoggedTunableNumber driveKI =
            new LoggedTunableNumber("drive/driveKI", TunerConstants.driveGains.kI);
    private final LoggedTunableNumber driveKD =
            new LoggedTunableNumber("drive/driveKD", TunerConstants.driveGains.kD);
    private final LoggedTunableNumber driveKS =
            new LoggedTunableNumber("drive/driveKS", TunerConstants.driveGains.kS);
    private final LoggedTunableNumber driveKV =
            new LoggedTunableNumber("drive/driveKV", TunerConstants.driveGains.kV);
    private final LoggedTunableNumber driveKA =
            new LoggedTunableNumber("drive/driveKA", TunerConstants.driveGains.kA);

    private final LoggedTunableNumber steerKP =
            new LoggedTunableNumber("drive/steerKP", TunerConstants.steerGains.kP);
    private final LoggedTunableNumber steerKI =
            new LoggedTunableNumber("drive/steerKI", TunerConstants.steerGains.kI);
    private final LoggedTunableNumber steerKD =
            new LoggedTunableNumber("drive/steerKD", TunerConstants.steerGains.kD);
    private final LoggedTunableNumber steerKS =
            new LoggedTunableNumber("drive/steerKS", TunerConstants.steerGains.kS);
    private final LoggedTunableNumber steerKV =
            new LoggedTunableNumber("drive/steerKV", TunerConstants.steerGains.kV);
    private final LoggedTunableNumber steerKA =
            new LoggedTunableNumber("drive/steerKA", TunerConstants.steerGains.kA);

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        this.drivetrainConstants = drivetrainConstants;

        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        new DriveSysId(this);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(pathApplyRobotSpeeds
                            .withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(
                                    feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
                    );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? RED_ALLIANCE_PERSPECTIVE_ROTATION
                                : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
                hasAppliedOperatorPerspective = true;
            });
        }

        Logger.recordOutput("drive/moduleTargets", getState().ModuleTargets);
        Logger.recordOutput("drive/moduleStates", getState().ModuleStates);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                PIDSVA -> {
                    var gains = new Slot0Configs()
                            .withKP(PIDSVA[0])
                            .withKI(PIDSVA[1])
                            .withKD(PIDSVA[2])
                            .withKS(PIDSVA[3])
                            .withKV(PIDSVA[4])
                            .withKA(PIDSVA[5]);

                    Arrays.stream(getModules())
                            .forEach(module ->
                                    module.getDriveMotor().getConfigurator().apply(gains));
                },
                driveKP,
                driveKI,
                driveKD,
                driveKS,
                driveKV,
                driveKA);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                PIDSVA -> {
                    var gains = new Slot0Configs()
                            .withKP(PIDSVA[0])
                            .withKI(PIDSVA[1])
                            .withKD(PIDSVA[2])
                            .withKS(PIDSVA[3])
                            .withKV(PIDSVA[4])
                            .withKA(PIDSVA[5]);

                    Arrays.stream(getModules())
                            .forEach(module ->
                                    module.getSteerMotor().getConfigurator().apply(gains));
                },
                steerKP,
                steerKI,
                steerKD,
                steerKS,
                steerKV,
                steerKA);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(
                visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @AutoLogOutput(key = "drive/heading")
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    @AutoLogOutput(key = "drive/pose")
    public Pose2d getPose() {
        return getState().Pose;
    }

    @AutoLogOutput(key = "drive/chassisSpeeds")
    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    @AutoLogOutput(key = "drive/fieldSpeeds")
    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getSpeeds(), getHeading());
    }

    private final PIDController trajectoryXController = new PIDController(14, 0, 0);
    private final PIDController trajectoryYController = new PIDController(14, 0, 0);
    private final PIDController trajectoryThetaController = new PIDController(3, 0, 0);

    private final SwerveRequest.ApplyFieldSpeeds trajectoryApplyFieldSpeeds =
            new SwerveRequest.ApplyFieldSpeeds()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public void followTrajectory(SwerveSample sample) {
        Logger.recordOutput("drive/trajectorySample", sample);

        trajectoryThetaController.enableContinuousInput(-Math.PI, Math.PI);
        var pose = getState().Pose;
        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += trajectoryXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += trajectoryYController.calculate(pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += trajectoryThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading);

        setControl(trajectoryApplyFieldSpeeds
                .withSpeeds(targetSpeeds)
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    /* Drive to Pose */

    /*
     * The below code is based on 6995 Nomad's drive to pose code.
     * https://github.com/frc6995/Robot-2025/blob/ec7eccecc7d6793f9a4bf5057c5cf5b1bfb0f65d/src/main/java/frc/robot/subsystems/DriveBaseS.java#L524
     */

    /* DRIVE TO POSE using Trapezoid Profiles */
    private TrapezoidProfile.Constraints driveToPoseConstraints =
            new TrapezoidProfile.Constraints(1, 1);
    private TrapezoidProfile.Constraints driveToPoseRotationConstraints =
            new TrapezoidProfile.Constraints(3, 6);
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
    TrapezoidProfile.State translationState = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State rotationState = new TrapezoidProfile.State(0, 0);

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
        Command command = runOnce(() -> {
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

                    var speeds = getFieldSpeeds();

                    // A negative velocity means we are approaching 0, the goal is 0
                    translationState.velocity = MathUtil.clamp(
                            VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                                    .dot(directionGoalToBot),
                            -driveToPoseConstraints.maxVelocity,
                            0);

                    Logger.recordOutput(
                            "drive/driveToPose/unclampedInitialVelocity",
                            VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                                    .dot(directionGoalToBot));

                    // Initial state of rotation
                    driveToPoseRotationGoal.position = goal.inner.getRotation().getRadians();

                    rotationState.position = initial.inner.getRotation().getRadians();
                    rotationState.velocity = speeds.omegaRadiansPerSecond;

                    Logger.recordOutput("drive/driveToPose/initial", initial.inner);
                    Logger.recordOutput("drive/driveToPose/goal", goal.inner);
                    Logger.recordOutput(
                            "drive/driveToPose/initialDistance", translationState.position);
                    Logger.recordOutput(
                            "drive/driveToPose/initialVelocity", translationState.velocity);
                })
                .andThen(run(() -> {
                    var setpoint = driveToPoseProfile.calculate(
                            Constants.LOOP_PERIOD_SEC, translationState, driveToPoseGoal);
                    translationState.position = setpoint.position;
                    translationState.velocity = setpoint.velocity;

                    Logger.recordOutput("drive/driveToPose/distance", translationState.position);
                    Logger.recordOutput("drive/driveToPose/velocity", translationState.velocity);

                    // I am trusting them here

                    // Rotation continuous input
                    // Get error which is the smallest distance between goal
                    // and measurement
                    double errorBound = Math.PI;
                    var measurement = getHeading().getRadians();
                    double goalMinDistance = MathUtil.inputModulus(
                            driveToPoseRotationGoal.position - measurement,
                            -errorBound,
                            errorBound);
                    double setpointMinDistance = MathUtil.inputModulus(
                            rotationState.position - measurement, -errorBound, errorBound);

                    // Recompute the profile goal with the smallest error,
                    // thus giving the shortest path. The goal
                    // may be outside the input range after this operation,
                    // but that's OK because the controller
                    // will still go there and report an error of zero. In
                    // other words, the setpoint only needs to
                    // be offset from the measurement by the input range
                    // modulus; they don't need to be equal.
                    driveToPoseRotationGoal.position = goalMinDistance + measurement;
                    rotationState.position = setpointMinDistance + measurement;

                    var rotSetpoint = driveToPoseRotationProfile.calculate(
                            0.02, rotationState, driveToPoseRotationGoal);
                    rotationState.position = rotSetpoint.position;
                    rotationState.velocity = rotSetpoint.velocity;

                    var startPose = initial.inner;

                    var interpTrans = goal.inner
                            .getTranslation()
                            .interpolate(
                                    startPose.getTranslation(), setpoint.position / distance.inner);

                    if (atDriveToPosePose.getAsBoolean()) {
                        setControl(pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
                    } else {
                        followTrajectory(DriveUtil.sample(
                                interpTrans,
                                new Rotation2d(rotationState.position),
                                normDirStartToEnd.inner.getX() * setpoint.velocity,
                                normDirStartToEnd.inner.getY() * setpoint.velocity,
                                rotationState.velocity));
                    }
                }))
                .until(atDriveToPosePose.debounce(.1))
                .finallyDo(() -> setControl(pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds())));

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
        return new Trigger(() -> {
            Pose2d pose = poseSup.get();
            Pose2d currentPose = getPose();
            boolean transValid = currentPose.getTranslation().getDistance(pose.getTranslation())
                    < toleranceMeters;
            boolean rotValid = withinTolerance(
                    currentPose.getRotation(), pose.getRotation(), toleranceRadians);
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

    /* END DRIVE TO POSE */

}
