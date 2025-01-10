package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.SupplierUtils.toRadians;
import static frc.robot.Constants.Arm.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.UnitDashboardNumber;
import frc.robot.Constants;
import frc.robot.Constants.Arm.FeedForwardConstants;
import frc.robot.Robot;
import frc.robot.subsystems.leds.Leds;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import lombok.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends BlitzSubsystem {
    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("Arm/kP", Constants.Arm.PidConstants.P);
    private final LoggedTunableNumber kI =
            new LoggedTunableNumber("Arm/kI", Constants.Arm.PidConstants.I);
    private final LoggedTunableNumber kD =
            new LoggedTunableNumber("Arm/kD", Constants.Arm.PidConstants.D);

    private final LoggedTunableNumber kS =
            new LoggedTunableNumber("Arm/kS", Constants.Arm.FeedForwardConstants.KS);
    private final LoggedTunableNumber kV =
            new LoggedTunableNumber("Arm/kV", Constants.Arm.FeedForwardConstants.KV);
    private final LoggedTunableNumber kA =
            new LoggedTunableNumber("Arm/kA", Constants.Arm.FeedForwardConstants.KA);
    private final LoggedTunableNumber kG =
            new LoggedTunableNumber("Arm/kG", Constants.Arm.FeedForwardConstants.KG);

    private static final LoggedTunableNumber maxRot =
            new LoggedTunableNumber("Arm/MaxRot", MAX_ROT);
    private static final LoggedTunableNumber transitNormal =
            new LoggedTunableNumber("Arm/MinRot", MIN_ROT);

    public enum State {
        CLOSED_LOOP,
        MANUAL,
        CHARACTERIZING
    }

    @NoArgsConstructor(force = true)
    @RequiredArgsConstructor
    public enum Goals {
        INTAKE(
                new LoggedTunableNumber("Arm/IntakeDegrees", Math.toDegrees(Positions.INTAKE)),
                true),
        CLIMB(new LoggedTunableNumber("Arm/ClimbDegrees", Math.toDegrees(Positions.CLIMB)), true),
        AMP_BACK(
                new LoggedTunableNumber("Arm/AmpBackDegrees", Math.toDegrees(Positions.AMP_FRONT))),
        AMP_FRONT(
                new LoggedTunableNumber("Arm/AmpFrontDegrees", Math.toDegrees(Positions.AMP_BACK))),
        SUBWOOFER(
                new LoggedTunableNumber(
                        "Arm/SubwooferDegrees", Math.toDegrees(Positions.SPEAKER_SUB_FRONT))),
        PODIUM(
                new LoggedTunableNumber(
                        "Arm/PodiumDegrees", Math.toDegrees(Positions.SPEAKER_PODIUM))),
        // Used for tuning/debugging, should be unused on field
        CUSTOM(new LoggedTunableNumber("Arm/CustomDegrees", 45)),

        // Non static external state
        TRANSIT(new LoggedTunableNumber("Arm/Transit", Math.toDegrees(Positions.TRANSIT_NORMAL))),
        AIM(arm -> Math.toDegrees(arm.aimGoal.getAsDouble()));

        private final Function<Arm, Double> armSetpoint;
        private final boolean letRest;

        // Some states require member fields of the subsystem, which do not exist at enum creation.
        private double getRads(Arm arm) {
            return Math.toRadians(armSetpoint != null ? armSetpoint.apply(arm) : Double.NaN);
        }

        // Only a few of our states require member variables of the static, the rest don't need it
        // and are defined as simple double suppliers.
        Goals(DoubleSupplier setpointSupplier) {
            this(setpointSupplier, false);
        }

        Goals(Function<Arm, Double> armSetpoint) {
            this(armSetpoint, false);
        }

        Goals(DoubleSupplier setpointSupplier, boolean letRest) {
            this(x -> setpointSupplier.getAsDouble(), letRest);
        }

        public <T> Map.Entry<Goals, T> asEntry(T value) {
            return Map.entry(this, value);
        }
    }

    /** represents a possible arm state. */
    private record ArmState(
            DoubleSupplier voltage, DoubleSupplier position, DoubleSupplier velocity) {
        private static ArmState fromVoltage(DoubleSupplier voltage) {
            return new ArmState(voltage, () -> Double.NaN, () -> Double.NaN);
        }

        private static ArmState of(DoubleSupplier position) {
            return of(position, () -> 0);
        }

        private static ArmState of(DoubleSupplier position, DoubleSupplier velocity) {
            return new ArmState(() -> Double.NaN, position, velocity);
        }

        private boolean isClosedLoop() {
            return !Double.isNaN(position.getAsDouble()) && !Double.isNaN(velocity.getAsDouble());
        }
    }

    private Map<Goals, DoubleSupplier> goalMap;

    @AutoLogOutput @Getter Goals goal = Goals.TRANSIT;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            Constants.Arm.MAX_VELOCITY, Constants.Arm.MAX_ACCELERATION));
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    @Setter private DoubleSupplier aimGoal = () -> 0;
    @Setter private BooleanSupplier stageSafety = () -> false;
    @Setter private BooleanSupplier disabledOverride = () -> false;
    @Setter private BooleanSupplier manualOverride = () -> false;
    @Setter private DoubleSupplier manualControl = () -> 0;

    private ArmFeedforward feedforward;

    private final SysIdRoutine routine;

    public Arm(ArmIO io) {
        super("arm");
        this.io = io;

        goalMap = new EnumMap<>(Goals.class);

        goalMap = Map.ofEntries(
                    Goals.INTAKE.asEntry(
                            UnitDashboardNumber.radiansDegrees("Arm/IntakeDegrees", Positions.INTAKE)),
                    Goals.CLIMB.asEntry(
                            UnitDashboardNumber.radiansDegrees("Arm/ClimbDegrees", Positions.CLIMB)
                    ), Goals.AMP_BACK.asEntry(
                            UnitDashboardNumber.radiansDegrees("Arm/AMP_BACKDegrees", Positions.AMP_BACK)));





        goalMap.put(
                Goals.INTAKE,
                toRadians(
                        new LoggedTunableNumber(
                                "Arm/IntakeDegrees", Units.radiansToDegrees(Positions.INTAKE))));

        //                stateMap = Map.ofEntries(
        //                        Goals.INTAKE.asEntry(ArmState.of(toRadians(new
        //         LoggedTunableNumber("Arm/IntakeDegrees", Positions.INTAKE)))),
        //                        Goals.CLIMB.asEntry(ArmState.of(toRadians(new
        //         LoggedTunableNumber("Arm/IntakeDegrees", Positions.INTAKE))))
        //                );

        //        INTAKE(new LoggedTunableNumber("Arm/IntakeDegrees", Positions.INTAKE), true),
        //                CLIMB(new LoggedTunableNumber("Arm/ClimbDegrees", Positions.CLIMB), true),
        //                AMP(new LoggedTunableNumber("Arm/AmpDegrees", Positions.AMP)),
        //                SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees",
        // Positions.SPEAKER_SUB_FRONT)),
        //                PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees",
        // Positions.SPEAKER_PODIUM)),
        //                // Used for tuning/debugging, should be unused on field
        //                CUSTOM(new LoggedTunableNumber("Arm/CustomDegrees", 45)),
        //
        //                // Non static external state
        //                TRANSIT(new LoggedTunableNumber("Arm/Transit", Positions.TRANSIT_NORMAL)),
        //                AIM(arm -> Math.toDegrees(arm.aimGoal.getAsDouble())),
        //
        //                // Special Cases, must be handled individually by subsystem periodic
        //                CHARACTERIZING,
        //                MANUAL;

        setDefaultCommand(setGoal(Goals.TRANSIT));

        feedforward =
                new ArmFeedforward(
                        FeedForwardConstants.KS,
                        FeedForwardConstants.KG,
                        FeedForwardConstants.KV,
                        FeedForwardConstants.KA);

        // Do this, but smarter
        new Trigger(() -> inputs.encoderConnected)
                .onTrue(
                        Commands.waitSeconds(.25)
                                .andThen(() -> io.seedArmPosition(false))
                                .andThen(io::stop)
                                .ignoringDisable(true));

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(5),
                                null,
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (Voltage volts) -> {
                                    System.out.println(volts.baseUnitMagnitude());
                                    io.setArmVolts(volts.in(Volts));
                                },
                                null, // No log consumer, since data is recorded by URCL
                                this));

        ShuffleboardTab tab = Shuffleboard.getTab("Sysid");
        tab.add("ArmQuasistaticFwd", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tab.add("ArmQuasistaticRev", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        tab.add("ArmDynamicFwd", sysIdDynamic(SysIdRoutine.Direction.kForward));
        tab.add("ArmDynamicRev", sysIdDynamic(SysIdRoutine.Direction.kReverse));

        ShuffleboardTab autoShootTab = Shuffleboard.getTab("AutoShoot");
        @SuppressWarnings("resource")
        GenericEntry testArm = autoShootTab.add("testArm", 0).getEntry();
        autoShootTab.add("custom arm", setGoal(Goals.CUSTOM));
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                kSGVA -> feedforward = new ArmFeedforward(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                kS,
                kG,
                kV,
                kA);

        io.seedArmPosition(false); // TODO, try removing this.

        if (DriverStation.isDisabled() || disabledOverride.getAsBoolean()) {
            // Reset profile when disabled
            setpointState = new TrapezoidProfile.State(inputs.rotation, 0);

            // Stop arm
            io.stop();
            return;
        }

        if (goal != Goals.MANUAL) {
            setpointState =
                    profile.calculate(
                            Robot.defaultPeriodSecs,
                            setpointState,
                            new TrapezoidProfile.State(
                                    MathUtil.clamp(
                                            goal.getRads(this),
                                            MIN_ROT,
                                            stageSafety.getAsBoolean() ? MAX_STAGE : MAX_ROT),
                                    0));

            if (goal.letRest
                    && atGoal()) // Let rest goals will just let the arm fall down once the setpoint
                // is reached.
                io.stop();
            else updateRotation(setpointState.position, setpointState.velocity);
        } else {
            //            setVoltage
        }
        // TODO, characterization should just work, but manual override still needs implementing
    }

    public void updateRotation(double degrees, double velocity) {
        Logger.recordOutput(logKey + "/wanted_rotation", degrees);
        Logger.recordOutput(logKey + "/wanted_velocity", velocity);
        io.setRotationSetpoint(degrees, feedforward.calculate(degrees, velocity));
    }

    public void setArmRotationSpeed(double percent) {
        io.setArmSpeed(percent);
    }

    public Command setGoal(@NonNull Arm.Goals goal) {
        return runEnd(() -> this.goal = goal, () -> this.goal = Goals.TRANSIT)
                .withName("Arm " + goal.toString().toLowerCase());
    }

    @AutoLogOutput
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(setpointState.position, goal.getRads(this), 1e-3);
    }

    /* SYSID STUFF */
    // Creates a SysIdRoutine
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction)
                //                .deadlineWith(setGoal(Goals.CHARACTERIZING))
                .withName(
                        logKey
                                + "/quasistatic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction)
                //                .deadlineWith(setGoal(Goals.CHARACTERIZING))
                .withName(
                        logKey
                                + "/dynamic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrake(false), () -> io.setBrake(true))
                .beforeStarting(() -> Leds.getInstance().armCoast = true)
                .finallyDo(() -> Leds.getInstance().armCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
