package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.NanUtil.TRAPEZOID_NAN_STATE;
import static frc.robot.Constants.Wrist.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

    private Optional<TrapezoidProfile.State> goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;

    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("wrist/kP", Constants.Wrist.PidGains.KP);
    private final LoggedTunableNumber kI =
            new LoggedTunableNumber("wrist/kI", Constants.Wrist.PidGains.KI);
    private final LoggedTunableNumber kD =
            new LoggedTunableNumber("wrist/kD", Constants.Wrist.PidGains.KD);

    private final LoggedTunableNumber kS =
            new LoggedTunableNumber("wrist/kS", Constants.Wrist.WristGains.KS);
    private final LoggedTunableNumber kV =
            new LoggedTunableNumber("wrist/kV", Constants.Wrist.WristGains.KV);
    private final LoggedTunableNumber kA =
            new LoggedTunableNumber("wrist/kA", Constants.Wrist.WristGains.KA);
    private final LoggedTunableNumber kG =
            new LoggedTunableNumber("wrist/kG", Constants.Wrist.WristGains.KG);

    Supplier<Command> superstructureIdleCommand;

    public Wrist(WristIO io, Supplier<Command> superstructureIdleCommand) {
        super("wrist");
        this.io = io;
        this.superstructureIdleCommand = superstructureIdleCommand;

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = Optional.empty();

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.per(Second).of(.5),
                                Units.Volts.of(4),
                                null,
                                Constants.compBot()
                                        ? (state) ->
                                                SignalLogger.writeString(
                                                        "sysid-wrist-state", state.toString())
                                        : null),
                        new SysIdRoutine.Mechanism(
                                (volts) -> io.setVolts(volts.in(Units.Volts)), null, this));

        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .withName("Wrist Quasistic Forward"));
        characterizationTab.add(
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .withName("Wrist Quasistic Reverse"));

        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kForward).withName("Wrist Dynamic Forward"));
        characterizationTab.add(
                sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("Wrist Dynamic Reverse"));

        characterizationTab.add(
                "wrist/45",
                withGoal(new TrapezoidProfile.State(Math.toRadians(45), 0))
                        .withName("wrist/test45"));
        characterizationTab.add(
                "wrist/minus45",
                withGoal(new TrapezoidProfile.State(Math.toRadians(-45), 0))
                        .withName("wrist/testminus45"));
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        if (goal.isPresent() && DriverStation.isEnabled()) {
            TrapezoidProfile.State future_setpoint =
                    profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal.get());

            if (Constants.compBot()) {
                // If on comp bot, use motion magic to get to goal, trapezoid is just a guide.
                io.setMotionMagic(goal.get().position);
            } else {
                // If on dev bot, set the onboard pid to the current trapezoid position.
                io.setSetpoint(setpoint.position, setpoint.velocity, future_setpoint.velocity);
            }

            setpoint = future_setpoint;
        }

        if (DriverStation.isDisabled()) {
            // Reset profile while disabled
            setpoint = new TrapezoidProfile.State(getPosition(), 0);
            goal = Optional.empty();

            // Stop arm
            io.stop();
        }

        Logger.recordOutput(logKey + "/profile/positionSetpoint", setpoint.position);
        Logger.recordOutput(logKey + "/profile/velocitySetpoint", setpoint.velocity);

        Logger.recordOutput(logKey + "/profile/goalPresent", goal.isPresent());
        Logger.recordOutput(
                logKey + "/profile/positionGoal", goal.orElse(TRAPEZOID_NAN_STATE).position);
        Logger.recordOutput(
                logKey + "/profile/velocityGoal", goal.orElse(TRAPEZOID_NAN_STATE).velocity);

        Logger.recordOutput(
                logKey + "/absEncoderDegrees", Math.toRadians(inputs.absoluteEncoderPosition));

        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), kP, kI, kD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                (kSGVA) -> io.setFF(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                kS,
                kG,
                kV,
                kA);
    }

    @AutoLogOutput(key = "wrist/position")
    public double getPosition() {
        return inputs.absoluteEncoderPosition;
    }

    @AutoLogOutput(key = "wrist/velocity")
    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
    }

    @AutoLogOutput(key = "wrist/idealPosition")
    public double getIdealPosition() {
        if (goal.isPresent()) {
            return setpoint.position;
        }
        return getPosition();
    }

    @AutoLogOutput(key = "wrist/idealVelocity")
    public double getIdealVelocity() {
        if (goal.isPresent()) {
            return setpoint.velocity;
        }
        return getVelocity();
    }

    public Command setSpeed(double speed) {
        return setSpeed(() -> speed);
    }

    public Command setSpeed(DoubleSupplier speed) {
        return runEnd(
                        () -> {
                            io.setPercent(speed.getAsDouble());
                        },
                        () -> {
                            io.setPercent(0);
                        })
                .beforeStarting(() -> this.goal = Optional.empty())
                .withName(logKey + "/speed");
    }

    public Command withGoal(TrapezoidProfile.State goal) {
        return goToPosition(goal.position, false).withName(logKey + "/withGoal " + goal.position);
    }

    /**
     * @param requireProfileCompletion If true, this command will only end once the mechanism
     *     position is within a tolerance of the wanted position. Else this command will end once
     *     the time to position has elapsed, with no guarantee that the mechanism is actually at the
     *     goal.
     * @return A command that moves the mechanism to the desired position with motion profiling and
     *     ends when the mechanism reaches that goal.
     */
    public Command goToPosition(double position, boolean requireProfileCompletion) {
        if (requireProfileCompletion)
            return followGoal(() -> position)
                    .withDeadline(
                            Commands.waitUntil(
                                    () -> MathUtil.isNear(position, getPosition(), TOLERANCE)))
                    .withName(logKey + "/goToPosition_waitForMechanism " + position);
        else
            return followGoal(() -> position)
                    .withDeadline(
                            Commands.waitUntil(
                                            () ->
                                                    MathUtil.isNear(
                                                            position, getIdealPosition(), 1e-9))
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
                })
                .handleInterrupt(() -> this.goal = Optional.of(setpoint))
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction).alongWith(superstructureIdleCommand.get());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction).alongWith(superstructureIdleCommand.get());
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true))
                .beforeStarting(() -> Leds.getInstance().superstructureCoast = true)
                .finallyDo(() -> Leds.getInstance().superstructureCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
