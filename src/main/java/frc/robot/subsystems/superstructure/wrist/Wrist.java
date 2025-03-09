package frc.robot.subsystems.superstructure.wrist;

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
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends BlitzSubsystem {
    public final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(Math.toRadians(180), Math.toRadians(360)));

    private TrapezoidProfile.State goal;
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

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;

        setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
        goal = null;

        ShuffleboardTab characterizationTab = Shuffleboard.getTab("Characterization");

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, Units.Volts.of(5), null),
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



        if (goal != null && DriverStation.isEnabled()) {
            TrapezoidProfile.State future_setpoint = profile.calculate(Constants.LOOP_PERIOD_SEC, setpoint, goal);

            io.setSetpoint(setpoint.position, setpoint.velocity, future_setpoint.velocity);

            Logger.recordOutput(logKey + "/profile/positionSetpoint", setpoint.position);
            Logger.recordOutput(logKey + "/profile/velocitySetpoint", setpoint.velocity);

            Logger.recordOutput(logKey + "/profile/positionGoal", goal.position);
            Logger.recordOutput(logKey + "/profile/velocityGoal", goal.velocity);

            setpoint = future_setpoint;
        }

        if (DriverStation.isDisabled()) {
            // Reset profile when disabled
            setpoint = new TrapezoidProfile.State(getPosition(), 0);
            goal = null;

            // Stop arm
            io.stop();
            return;
        }

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
        return inputs.positionRadians;
    }

    @AutoLogOutput(key = "wrist/velocity")
    public double getVelocity() {
        return inputs.velocityRadiansPerSecond;
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
                .beforeStarting(() -> this.goal = null)
                .withName(logKey + "/speed");
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
        return runOnce(() -> setpoint = new TrapezoidProfile.State(getPosition(), 0));
        //                .onlyIf(
        //                        () ->
        //                                setpoint == null
        //                                        || !EqualsUtil.epsilonEquals(
        //                                        setpoint.position, getPosition(),
        // Math.toRadians(2))
        //                                        || !EqualsUtil.epsilonEquals(
        //                                        setpoint.velocity, getVelocity(),
        // Math.toRadians(4)));
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
