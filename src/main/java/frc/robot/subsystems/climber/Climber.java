package frc.robot.subsystems.climber;

import static frc.robot.Constants.Climber.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import org.littletonrobotics.junction.Logger;

public class Climber extends BlitzSubsystem {
    private final ClimberIO io;
    // TODO Make Auto Logs Work
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public Climber(ClimberIO io) {
        super("Climber");
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);
    }

    public Command deployClimber() {
        // TODO FIX
        return runOnce(() -> io.setMotionProfile(CLIMBER_DEPLOY))
                .andThen(
                        Commands.waitUntil(
                                () ->
                                        EqualsUtil.epsilonEquals(
                                                inputs.position, CLIMBER_DEPLOY, 1e-3)));
    }

    public Command climb() {
        // TODO repair
        return runOnce(() -> io.setMotionProfile(CLIMB))
                .andThen(
                        Commands.waitUntil(
                                () -> EqualsUtil.epsilonEquals(inputs.position, CLIMB, 1e-3)));
    }

    public Command restowClimber() {
        // TODO restore
        return runOnce(() -> io.setMotionProfile(RESTOW_CLIMBER))
                .andThen(
                        Commands.waitUntil(
                                () ->
                                        EqualsUtil.epsilonEquals(
                                                inputs.position, RESTOW_CLIMBER, 1e-3)));
    }

    public Command setSpeed(double speed) {
        return runEnd(
                () -> io.setSpeed(speed),
                () -> io.setSpeed(0)
        );
    }
}
