package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.SuperstructureSetpoints.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.lib.util.ScoringPositions;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;

/**
 * Welcome to the war room. (that's also what I called my college essay Google Drive but that's
 * another story)
 *
 * <p>you might be wondering how the hell we control this abomination of a superstructure that
 * mechanical designed, and answer is we don't and go on strike until they do a passthough.
 *
 * <p>Real answer: probably trigger voodo magic
 *
 * <p>idk how, ask me when im awake okay
 *
 * <p>dw it hasn't gotten to the point of 1am coding, thats still to come.
 *
 * <p>-Noah
 */
public class Superstructure extends BlitzSubsystem {
    private final Elevator elevator;
    private final Wrist wrist;

    public Superstructure(Elevator elevator, Wrist wrist) {
        super("superstructure");
        this.elevator = elevator;
        this.wrist = wrist;
    }

    // THIS REALLY SUCKS BUT WORKS AS A PLACEHOLDER IG
    public enum States {
        STOW,
        HANDOFF, // TODO: DEPRECATE ONCE DESIGN IS BETTER
        L1,
        L2,
        L3,
        L4
    }

    public Command stowCommand() {
        return Commands.sequence(
                elevator.withGoal(HANDOFF_PRIME.getElevatorState())
                        .onlyIf(
                                () ->
                                        elevator.getPosition() < HANDOFF_PRIME.elevatorPosition()
                                                && wrist.getPosition() < WRIST_HANDOFF_DANGER_ZONE),
                Commands.parallel(
                        wrist.withGoal(STOW.getWristState()),
                        elevator.withGoal(STOW.getElevatorState())
                                .beforeStarting(
                                        Commands.waitUntil(
                                                () ->
                                                        wrist.getPosition()
                                                                > ELEVATOR_DOWN_WRIST_MIN))));
    }
    

    public Command primeHandoff() {
        throw new UnsupportedOperationException();
    }

    public Command executeHandoff() {
        throw new UnsupportedOperationException();
    }

    public Command primeScore(ScoringPositions.Level level) {
        throw new UnsupportedOperationException();
    }

    public Command executeScore(ScoringPositions.Level level) {
        throw new UnsupportedOperationException();
    }
}
