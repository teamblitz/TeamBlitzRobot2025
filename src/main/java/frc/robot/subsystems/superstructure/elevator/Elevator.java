package frc.robot.subsystems.superstructure.elevator;

import static frc.lib.util.NanUtil.TRAPEZOID_NAN_STATE;
import static frc.robot.Constants.Elevator.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends BlitzSubsystem {
    private final frc.robot.subsystems.superstructure.elevator.ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private Optional<TrapezoidProfile.State> goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;
    private final Supplier<Command> superstructureIdle;

    private final Timer loopTimer = new Timer();

    // Left Elevator Tunable Numbers
    private final LoggedTunableNumber leftKP =
            new LoggedTunableNumber("elevator/leftKP", Constants.Elevator.LeftGains.KP);
    private final LoggedTunableNumber leftKI =
            new LoggedTunableNumber("elevator/leftKI", Constants.Elevator.LeftGains.KI);
    private final LoggedTunableNumber leftKD =
            new LoggedTunableNumber("elevator/leftKD", Constants.Elevator.LeftGains.KD);

    private final LoggedTunableNumber leftKS =
            new LoggedTunableNumber("elevator/leftKS", Constants.Elevator.LeftGains.KS);
    private final LoggedTunableNumber leftKV =
            new LoggedTunableNumber("elevator/leftKV", Constants.Elevator.LeftGains.KV);
    private final LoggedTunableNumber leftKA =
            new LoggedTunableNumber("elevator/leftKA", Constants.Elevator.LeftGains.KA);
    private final LoggedTunableNumber leftKG =
            new LoggedTunableNumber("elevator/leftKG", Constants.Elevator.LeftGains.KG);

    // Right Elevator Tunable Numbers
    private final LoggedTunableNumber rightKP =
            new LoggedTunableNumber("elevator/rightKP", Constants.Elevator.RightGains.KP);
    private final LoggedTunableNumber rightKI =
            new LoggedTunableNumber("elevator/rightKI", Constants.Elevator.RightGains.KI);
    private final LoggedTunableNumber rightKD =
            new LoggedTunableNumber("elevator/rightKD", Constants.Elevator.RightGains.KD);

    private final LoggedTunableNumber rightKS =
            new LoggedTunableNumber("elevator/rightKS", Constants.Elevator.RightGains.KS);
    private final LoggedTunableNumber rightKV =
            new LoggedTunableNumber("elevator/rightKV", Constants.Elevator.RightGains.KV);
    private final LoggedTunableNumber rightKA =
            new LoggedTunableNumber("elevator/rightKA", Constants.Elevator.RightGains.KA);
    private final LoggedTunableNumber rightKG =
            new LoggedTunableNumber("elevator/rightKG", Constants.Elevator.RightGains.KG);

    public Elevator(ElevatorIO io, Supplier<Command> superstructureIdle) {
        super("elevator");
        this.io = io;
        this.superstructureIdle = superstructureIdle;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = Optional.empty();

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.per(Units.Seconds).of(.5),
                                Units.Volts.of(4),
                                null,
                                Constants.compBot()
                                        ? (state) ->
                                                SignalLogger.writeString(
                                                        "sysid-elevator-state", state.toString())
                                        : null),
                        new SysIdRoutine.Mechanism(
                                (volts) -> io.setVolts(volts.in(Units.Volts)), null, this));

        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("Elevator Quasistic Forward"));
        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("Elevator Quasistic Reverse"));

        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kForward).withName("Elevator Dynamic Forward"));
        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("Elevator Dynamic Reverse"));

        characterizationTab.add(
                "elevator/0.1m",
                withGoal(new TrapezoidProfile.State(.1, 0))
                        .alongWith(superstructureIdle.get())
                        .withName("elevator/0.1m test"));

        characterizationTab.add(
                "elevator/0.4m",
                withGoal(new TrapezoidProfile.State(.8, 0))
                        .alongWith(superstructureIdle.get())
                        .withName("elevator/0.5m test"));

        loopTimer.restart();
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        if (goal.isPresent() && DriverStation.isEnabled()) {
            setpoint = profile.calculate(loopTimer.get(), setpoint, goal.get());
            TrapezoidProfile.State future_setpoint = profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal.get());


            if (Constants.compBot()) {
                // If on comp bot, use motion magic to get to goal, trapezoid is just a guide.
                io.setMotionMagic(goal.get().position);
            } else {
                // If on dev bot, set the onboard pid to the current trapezoid position.
                io.setSetpoint(setpoint.position, setpoint.velocity, future_setpoint.velocity);
            }
        }

        if (DriverStation.isDisabled()) {
            // Reset profile while disabled
            setpoint = new TrapezoidProfile.State(getPosition(), 0);
            goal = Optional.empty();

            // Stop arm
            io.stop();
        }

        loopTimer.reset();

        Logger.recordOutput(logKey + "/profile/positionSetpoint", setpoint.position);
        Logger.recordOutput(logKey + "/profile/velocitySetpoint", setpoint.velocity);

        Logger.recordOutput(logKey + "/profile/goalPresent", goal.isPresent());
        Logger.recordOutput(
                logKey + "/profile/positionGoal", goal.orElse(TRAPEZOID_NAN_STATE).position);
        Logger.recordOutput(
                logKey + "/profile/velocityGoal", goal.orElse(TRAPEZOID_NAN_STATE).velocity);

        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setPidLeft(pid[0], pid[1], pid[2]), leftKP, leftKI, leftKD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (kSGVA) -> io.setFFLeft(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                leftKS,
                leftKG,
                leftKV,
                leftKA);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                pid -> io.setPidRight(pid[0], pid[1], pid[2]),
                rightKP,
                rightKI,
                rightKD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (kSGVA) -> io.setFFRight(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                rightKS,
                rightKG,
                rightKV,
                rightKA);
    }

    public Command withSpeed(DoubleSupplier speed) {
        return runEnd(
                        () -> {
                            io.setSpeed(
                                    MathUtil.clamp(
                                            speed.getAsDouble(),
                                            atBottomLimit() ? 0 : -1,
                                            atTopLimit() ? 0 : 1));
                        },
                        () -> {
                            io.setSpeed(0);
                        })
                .beforeStarting(() -> this.goal = Optional.empty());
    }

    public Command withSpeed(double speed) {
        return withSpeed(() -> speed);
    }

    public Command upManual() {
        return withSpeed(0.1).withName("Up Manual");
    }

    public Command downManual() {
        return withSpeed(-0.1).withName("downManual");
    }

    public Command withGoal(TrapezoidProfile.State goal) {
        return goToPosition(goal.position, false).withName(logKey + "/withGoal " + goal.position);
    }

    /**
     * @param requireProfileCompletion If true, this command will only end once the mechanism position is within a
     *                                 tolerance of the wanted position. Else this command will end once the time to position has
     *                                 elapsed, with no guarantee that the mechanism is actually at the goal.
     * @return A command that moves the mechanism to the desired position with motion profiling and
     *     ends when the mechanism reaches that goal.
     */
    public Command goToPosition(double position, boolean requireProfileCompletion) {
        if (requireProfileCompletion)
            return followGoal(() -> position).withDeadline(
                            Commands.waitUntil(
                                    () -> MathUtil.isNear(position, getPosition(), TOLERANCE)))
                    .withName(logKey + "/goToPosition_waitForMechanism " + position);
        else
            return followGoal(() -> position)
                    .withDeadline(
                            Commands.waitUntil(
                                    () -> MathUtil.isNear(position, getIdealPosition(), 1e-9))
                                    .withName(logKey + "/goToPosition_waitForProfile " + position));

    }

    /**
     * @return A command that commands the mechanism to follow the provided goal and never ends.
     */
    public Command followGoal(DoubleSupplier goal) {
        return run(() -> {
            // Only update the goal if necessary to avoid GC overhead
            if (this.goal.isEmpty() || this.goal.get().position != goal.getAsDouble()) {
                this.goal =
                        Optional.of(
                                new TrapezoidProfile.State(
                                        MathUtil.clamp(
                                                goal.getAsDouble(), MIN_POS, MAX_POS),
                                        0));
            }
        }).handleInterrupt(() -> this.goal = Optional.of(setpoint))
                .beforeStarting(refreshCurrentState());
    }

    /**
     * Generates a command to reset the current internal setpoint to the actual state if they
     * conflict
     */
    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(() -> setpoint == null || goal.isEmpty());
    }

    @AutoLogOutput(key = "elevator/position")
    public double getPosition() {
        return inputs.position;
    }

    @AutoLogOutput(key = "elevator/velocity")
    public double getVelocity() {
        return (inputs.velocityLeft + inputs.velocityRight) / 2;
    }

    /**
     * @return The "ideal" position of the elevator
     */
    @AutoLogOutput(key = "elevator/idealPosition")
    public double getIdealPosition() {
        if (goal.isPresent()) {
            return setpoint.position;
        }
        return getPosition();
    }

    @AutoLogOutput(key = "elevator/idealVelocity")
    public double getIdealVelocity() {
        if (goal.isPresent()) {
            return setpoint.velocity;
        }
        return getVelocity();
    }


    public boolean atBottomLimit() {
        return Constants.devBot() && inputs.bottomLimitSwitch;
    }

    public boolean atTopLimit() {
        return Constants.devBot() && inputs.topLimitSwitch;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction).deadlineFor(superstructureIdle.get());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction).deadlineFor(superstructureIdle.get());
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true))
                .beforeStarting(() -> Leds.getInstance().superstructureCoast = true)
                .finallyDo(() -> Leds.getInstance().superstructureCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
