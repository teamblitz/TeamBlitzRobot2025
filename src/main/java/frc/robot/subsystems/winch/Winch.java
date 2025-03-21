package frc.robot.subsystems.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Winch.*;

public class Winch extends BlitzSubsystem {
    private final WinchIO io;
    private final WinchInputsAutoLogged inputs = new WinchInputsAutoLogged();

    private final LoggedTunableNumber maxOut =
            new LoggedTunableNumber("winch/maxOut", MAX_OUT);
    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("winch/kP", KP);

    private final LoggedTunableNumber matchFunnelUp =
            new LoggedTunableNumber("winch/matchFunnelUp", MATCH_FUNNEL_UP);

    private final LoggedTunableNumber matchFunnelDown =
        new LoggedTunableNumber("winch/matchFunnelDown", MATCH_FUNNEL_DOWN);

    private final LoggedTunableNumber pitFunnelStow =
            new LoggedTunableNumber("winch/pitFunnelStow", PIT_FUNNEL_STOW);


    public boolean ahh = false;

    public Winch(WinchIO io) {
        super("winch");
        this.io = io;

        ShuffleboardTab winchTab = Shuffleboard.getTab("winch");
        winchTab.add(raiseFunnel());
        winchTab.add(lowerFunnel());
        winchTab.add(pitFunnelReady());

        new Trigger(DriverStation::isDSAttached).onTrue(
                runOnce(
                        () -> {
                            if (!ahh) {
                                io.setPosition(PIT_FUNNEL_STOW);
                                ahh = true;
                            }
                        }
                )
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(), p -> io.setPid(p[0], 0, 0), kP);

        LoggedTunableNumber.ifChanged(
                hashCode(), OUT -> io.setMaxOutput(OUT[0]), maxOut);

    }

    public Command manualUp() {
        return runEnd(() -> io.setSpeed(.25), () -> io.setSpeed(0));
    }

    public Command manualDown() {
        return runEnd(() -> io.setSpeed(-.25), () -> io.setSpeed(0));
    }

    private Command goToPosition(DoubleSupplier position) {
        return new FunctionalCommand(
                () -> io.setMotionProfile(position.getAsDouble()),
                () -> {},
                (interrupted) -> io.setSpeed(0),
                () -> MathUtil.isNear(position.getAsDouble(), inputs.position, EPSILON),
                this
        );
    }

    // Designed for match use
    public Command raiseFunnel() {
        return goToPosition(matchFunnelUp).withName(logKey + "/raiseFunnel");
    }

    // For match use
    public Command lowerFunnel() {
        return goToPosition(matchFunnelDown).withName(logKey + "/lowerFunnel");
    }

    public Command pitFunnelReady() {
        return runOnce(() -> io.setPosition(0)).andThen(
                goToPosition(pitFunnelStow)).withName(logKey + "/pitFunnelReady");
    }
}
