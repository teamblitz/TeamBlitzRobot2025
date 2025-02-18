package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BlitzSubsystem;
import frc.lib.util.ScoringPositions;

/**
 * Welcome to the war room. (that's also what I called my college essay google drive but that's
 * another story)
 *
 * <p>you might be wondering how the hell we control this abomination of a superstructure that
 * mechanical designed, and answer is we dont and go on strike until they do a passthough.
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

    public Superstructure() {
        super("superstructure");
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

    private record SuperStructureState(double elevatorPos, double wristPos) {}
}
