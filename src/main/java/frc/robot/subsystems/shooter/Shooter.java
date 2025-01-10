package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
// import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends BlitzSubsystem {
    private final LoggedTunableNumber topKP =
            new LoggedTunableNumber("shooter/top/kP", Constants.Shooter.Spark.PID_TOP_P);
    private final LoggedTunableNumber topKI =
            new LoggedTunableNumber("shooter/top/kI", Constants.Shooter.Spark.PID_TOP_I);
    private final LoggedTunableNumber topKD =
            new LoggedTunableNumber("shooter/top/kD", Constants.Shooter.Spark.PID_TOP_D);

    private final LoggedTunableNumber bottomKP =
            new LoggedTunableNumber("shooter/bottom/kP", Constants.Shooter.Spark.PID_BOTTOM_P);
    private final LoggedTunableNumber bottomKI =
            new LoggedTunableNumber("shooter/bottom/kI", Constants.Shooter.Spark.PID_BOTTOM_I);
    private final LoggedTunableNumber bottomKD =
            new LoggedTunableNumber("shooter/bottom/kD", Constants.Shooter.Spark.PID_BOTTOM_D);

    private final LoggedTunableNumber topKS =
            new LoggedTunableNumber("shooter/top/kS", Constants.Shooter.Spark.FF_TOP_KS);
    private final LoggedTunableNumber topKV =
            new LoggedTunableNumber("shooter/top/kV", Constants.Shooter.Spark.FF_TOP_KV);
    private final LoggedTunableNumber topKA =
            new LoggedTunableNumber("shooter/top/kA", Constants.Shooter.Spark.FF_TOP_KA);

    private final LoggedTunableNumber bottomKS =
            new LoggedTunableNumber("shooter/bottom/kS", Constants.Shooter.Spark.FF_BOTTOM_KS);
    private final LoggedTunableNumber bottomKV =
            new LoggedTunableNumber("shooter/bottom/kV", Constants.Shooter.Spark.FF_BOTTOM_KV);
    private final LoggedTunableNumber bottomKA =
            new LoggedTunableNumber("shooter/bottom/kA", Constants.Shooter.Spark.FF_BOTTOM_KA);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpoint;

    private final SysIdRoutine routine;

    public Shooter(ShooterIO io) {
        super("shooter");
        this.io = io;

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> {
                                    io.setVolts(volts.in(Volts));
                                },
                                null,
                                this));

        ShuffleboardTab tab = Shuffleboard.getTab("Sysid");
        tab.add("ShooterQuasistaticFwd", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tab.add("ShooterQuasistaticRev", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        tab.add("ShooterDynamicFwd", sysIdDynamic(SysIdRoutine.Direction.kForward));
        tab.add("ShooterDynamicRev", sysIdDynamic(SysIdRoutine.Direction.kReverse));

        ShuffleboardTab autoShootTab = Shuffleboard.getTab("AutoShoot");
        GenericEntry testSpeed = autoShootTab.add("testSpeed", 0).getEntry();
        autoShootTab.add("testSpeedCmd", this.shootClosedLoopCommand(() -> testSpeed.getDouble(0)));
    }

    public void shootOpenLoop() {
        io.setPercent(1); // TODO CONST
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setTopPid(pid[0], pid[1], pid[2]), topKP, topKI, topKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                pid -> io.setBottomPid(pid[0], pid[1], pid[2]),
                bottomKP,
                bottomKI,
                bottomKD);
        LoggedTunableNumber.ifChanged(
                hashCode(), kSVA -> io.setTopFF(kSVA[0], kSVA[1], kSVA[2]), topKS, topKV, topKA);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                kSVA -> io.setBottomFF(kSVA[0], kSVA[1], kSVA[2]),
                bottomKS,
                bottomKV,
                bottomKA);
    }

    public void shootClosedLoop(double metersPerSecond) {
        setpoint = metersPerSecond;
        io.setSetpoint(metersPerSecond); // TODO, CONST
        Logger.recordOutput(logKey + "/velocitySetpoint");
    }

    private void reverse() {
        io.setPercent(-0.3);
    }

    private void stop() {
        io.setPercent(0);
    }

    public Command setSpeedCommand(double speed) {
        return runEnd(() -> io.setPercent(speed), this::stop).withName(logKey + "/speed " + speed);
    }

    public Command shootCommand() {
        return startEnd(this::shootOpenLoop, this::stop).withName(logKey + "/shoot");
    }

    public Command reverseCommand() {
        return startEnd(this::reverse, this::stop).withName(logKey + "/eject");
    }

    public Command shootClosedLoopCommand(DoubleSupplier metersPerSecond) {
        return run(() -> shootClosedLoop(metersPerSecond.getAsDouble()))
                .finallyDo(this::stop)
                .withName(logKey + "/closedLoopShoot");
    }

    public Command shootClosedLoopCommand(double metersPerSecond) {
        return shootClosedLoopCommand(() -> metersPerSecond);
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(setpoint, inputs.rpmTop, Constants.Shooter.MAX_VELOCITY * .01)
                && MathUtil.isNear(
                        setpoint, inputs.rpmBottom, Constants.Shooter.MAX_VELOCITY * .01);
    }

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
}
