package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.SuperstructureSetpoints.*;
import static java.util.Map.entry;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Map;
import java.util.Set;

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


        staticGoals = Map.ofEntries(
                entry(Goal.L1, L1),
                entry(Goal.L2, L2),
                entry(Goal.L3, L3),
                entry(Goal.L4, L4),
                entry(Goal.KICK_LOW_ALGAE, KICK_LOW_ALGAE),
                entry(Goal.KICK_HIGH_ALGAE, KICK_HIGH_ALGAE),

                entry(Goal.HANDOFF, HANDOFF),
                entry(Goal.STOW, STOW)


        );

        dynamicGoals = Map.ofEntries(
                entry(Goal.L4_DUNK, L4_DUNK)
        );

        ShuffleboardTab tab = Shuffleboard.getTab("SuperStructure");
        GenericEntry elevatorTestEntry = tab.add("elevatorTest", 0).getEntry();
        GenericEntry wristTestEntry = tab.add("wristTest", 45).getEntry();

        tab.add("positionTest",
                Commands.defer(
                        () -> Commands.parallel(
                                elevator.withGoal(new TrapezoidProfile.State(elevatorTestEntry.getDouble(0), 0)),
                                wrist.withGoal(new TrapezoidProfile.State(Math.toRadians(wristTestEntry.getDouble(45)), 0))
                        ).finallyDo(
                                (interrupt) -> Commands.print("SuperClosedTestEnd, " + interrupt)
                        ), Set.of(this)).withName("SuperStructure/closedTest")
                );

    }

    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput("superstructure/currentGoal", currentGoal);
        Logger.recordOutput("superstructure/previousGoal", previousGoal);

        Logger.recordOutput("superstructure/state", state);
    }

    private State state = State.UNKNOWN;
    private Goal previousGoal = null;
    private Goal currentGoal = Goal.STOW;

    private final Map<Goal, SuperstructureState> staticGoals;
    private final Map<Goal, List<SuperstructureState>> dynamicGoals;

    // THIS REALLY SUCKS BUT WORKS AS A PLACEHOLDER IG
    public enum Goal {
        STOW,
        HANDOFF,
        L1,
        L2,
        L3,
        L4,
        L4_DUNK,
        KICK_LOW_ALGAE,
        KICK_HIGH_ALGAE,
    }

    public enum State {
        // We are at a wanted goal, and not in between goals
        // The system is either in a desired position or desired sequence
        // A sequence (like l4 dunk) is considered at goal, not in transit
        AT_GOAL,
        // The system is in transit between two goals, IE stow and L4.
        // When in transit state, it is the job of the superstructure safely get between goals.
        IN_TRANSIT,
        // The system is not in transit between 2 known goals or at a goal
        // This is triggered whe either:
        //  - Control is shifted to the superstructure after a manual override
        //  - The robot is enabled and not in starting position
        //  - A sensor fault occurred.
        // While in this state it is the job of the superstructure to recover the mechanism to a known safe position
        // Failing this, safely disengage
        UNKNOWN,
        // Either a manual override or safety interlock triggered
        // superstructure control is disabled until robot is disabled and re-enabled, or manual re-enablement occurs
        DISABLED
    }

    public Command stowCommand() {
        return Commands.sequence(
                wrist.withGoal(STOW.getWristState()),
                elevator.withGoal(STOW.getElevatorState())
                .beforeStarting(
                        () -> {
                            currentGoal = Goal.STOW;
                            state = State.IN_TRANSIT;
                        }
                ).finallyDo(
                        (interrupted) -> state = interrupted ? State.UNKNOWN : State.AT_GOAL
                )).alongWith(idle())
                .withName("stow");

    }



    private Command toGoalWristLast(Goal goal) {
        return Commands.sequence(
            elevator.withGoal(
                staticGoals.get(goal).getElevatorState()
            ),
            wrist.withGoal(
                    staticGoals.get(goal).getWristState()
            ),
            Commands.idle(this)
        );
    }

    private Command toGoalWristFirst(Goal goal) {
        return Commands.sequence(
                wrist.withGoal(
                        staticGoals.get(goal).getWristState()
                ),
                elevator.withGoal(
                        staticGoals.get(goal).getElevatorState()
                ),
                Commands.idle(this)
        );
    }

    private Command toGoalSynchronous(Goal goal) {
        return Commands.parallel(
                elevator.withGoal(
                        staticGoals.get(goal).getElevatorState()
                ),
                wrist.withGoal(
                        staticGoals.get(goal).getWristState()
                ),
                Commands.idle(this)
        );
    }

    public Command toGoal(Goal goal) {
        if (goal == Goal.L4_DUNK) {
            Command command = idle();

            for (SuperstructureState state : dynamicGoals.get(goal)) {
                command = command.andThen(
                        Commands.parallel(
                                elevator.withGoal(
                                        state.getElevatorState()
                                ),
                                wrist.withGoal(
                                        state.getWristState()
                                ),
                                Commands.idle(this)
                        ));
            }

            return command.onlyIf(() -> currentGoal == Goal.L4);
        }


        return toGoalWristLast(goal)
                .beforeStarting(
                        () -> {
                            state = State.IN_TRANSIT;
                            previousGoal = currentGoal;
                            currentGoal = goal;
                        })
                .finallyDo(
                        (interrupted) -> {
                            state = interrupted ? State.UNKNOWN : State.AT_GOAL;
                        }
                );


    }

    public boolean atGoal(Goal goal) {
        return currentGoal == goal && state == State.AT_GOAL;
    }

    public Trigger triggerAtGoal(Goal goal) {
        return currentGoal == goal && state == State.AT_GOAL;
    }

    public Command idle() {
        return Commands.idle(this);
    }
}
