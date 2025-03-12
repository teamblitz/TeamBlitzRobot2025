package frc.robot.subsystems.superstructure.elevator;

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
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends BlitzSubsystem {
    private final frc.robot.subsystems.superstructure.elevator.ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 2);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private final SysIdRoutine routine;

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

    public Elevator(ElevatorIO io) {
        super("elevator");
        this.io = io;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = null;

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, Units.Volts.of(5), null),
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
                withGoal(new TrapezoidProfile.State(.1, 0)).withName("elevator/0.1m test"));

        characterizationTab.add(
                "elevator/0.4m",
                withGoal(new TrapezoidProfile.State(.8, 0)).withName("elevator/0.5m test"));

        loopTimer.restart();
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/rotLeftDeg", Math.toDegrees(inputs.positionLeft) % 360);
        Logger.recordOutput(logKey + "/rotRightDeg", Math.toDegrees(inputs.positionRight) % 360);

        if (goal != null && DriverStation.isEnabled()) {
            setpoint = profile.calculate(loopTimer.get(), setpoint, goal);
            TrapezoidProfile.State future_setpoint =
                    profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal);

            io.setSetpoint(setpoint.position, setpoint.velocity, future_setpoint.velocity);

            Logger.recordOutput(logKey + "/profile/positionSetpoint", setpoint.position);
            Logger.recordOutput(logKey + "/profile/velocitySetpoint", setpoint.velocity);

            Logger.recordOutput(logKey + "/profile/positionGoal", goal.position);
            Logger.recordOutput(logKey + "/profile/velocityGoal", goal.velocity);
        }

        if (DriverStation.isDisabled()) {
            // Reset profile when disabled
            setpoint = new TrapezoidProfile.State(getPosition(), 0);
            goal = null;

            // Stop arm
            io.stop();
            return;
        }

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

        loopTimer.reset();
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
                .beforeStarting(() -> this.goal = null);
    }

    public Command withSpeed(double speed) {
        return withSpeed(() -> speed);
    }

    public Command upManual() {
        return withSpeed(0.6).withName("Up Manual");
    }

    public Command downManual() {
        return withSpeed(-0.4).withName("downManual");
    }

    public Command withGoal(TrapezoidProfile.State goal) {
        return runOnce(
                        () -> {
                            this.goal = goal;
                        })
                .andThen(Commands.waitUntil(() -> setpoint.equals(goal)))
                .handleInterrupt(() -> this.goal = setpoint)
                .beforeStarting(refreshCurrentState());
    }

    /**
     * Generates a command to reset the current internal setpoint to the actual state if they
     * conflict
     */
    private Command refreshCurrentState() {
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), getVelocity()))
                .onlyIf(() -> setpoint == null);
    }

    @AutoLogOutput(key = "elevator/position")
    public double getPosition() {
        return (inputs.positionLeft + inputs.positionRight) / 2;
    }

    @AutoLogOutput(key = "elevator/velocity")
    public double getVelocity() {
        return (inputs.velocityLeft + inputs.velocityRight) / 2;
    }

    // TODO: Implement limit switch override
    public boolean atBottomLimit() {
        return inputs.bottomLimitSwitch;
    }

    public boolean atTopLimit() {
        return inputs.topLimitSwitch;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrakeMode(false), () -> io.setBrakeMode(true))
                .beforeStarting(() -> Leds.getInstance().armCoast = true)
                .finallyDo(() -> Leds.getInstance().armCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
