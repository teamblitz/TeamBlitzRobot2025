package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.SuperstructureSetpoints.*;

import static java.util.Map.entry;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;

import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;

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
    @Getter
    private final Elevator elevator;

    @Getter
    private final Wrist wrist;

    public Superstructure(ElevatorIO elevatorIO, WristIO wristIO) {

        super("superstructure");
        this.elevator = new Elevator(elevatorIO, this::idle);
        this.wrist = new Wrist(wristIO, this::idle);

        staticGoals = Map.ofEntries(
                entry(Goal.L1, L1),
                entry(Goal.L2, L2),
                entry(Goal.L3, L3),
                entry(Goal.L4, L4),
                entry(Goal.L4_PLOP, L4_PLOP),
                entry(Goal.KICK_LOW_ALGAE, KICK_LOW_ALGAE),
                entry(Goal.KICK_HIGH_ALGAE, KICK_HIGH_ALGAE),
                entry(Goal.HANDOFF, HANDOFF),
                entry(Goal.STOW, STOW));

        dynamicGoals = Map.ofEntries(
                entry(Goal.L4_DUNK, L4_DUNK)
                //                entry(Goal.L4_PLOP, L4_PLOP)
                );

        ShuffleboardTab tab = Shuffleboard.getTab("SuperStructure");
        GenericEntry elevatorTestEntry = tab.add("elevatorTest", 0).getEntry();
        GenericEntry wristTestEntry = tab.add("wristTest", 45).getEntry();

        tab.add(
                "positionTest",
                Commands.defer(
                                () -> Commands.parallel(
                                                elevator.withGoal(new TrapezoidProfile.State(
                                                        elevatorTestEntry.getDouble(0), 0)),
                                                wrist.withGoal(new TrapezoidProfile.State(
                                                        Math.toRadians(
                                                                wristTestEntry.getDouble(45)),
                                                        0)))
                                        .finallyDo((interrupt) ->
                                                Commands.print("SuperClosedTestEnd, " + interrupt)),
                                Set.of(this))
                        .withName("SuperStructure/closedTest"));
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
    private final Map<Goal, List<StateWithMode>> dynamicGoals;

    // THIS REALLY SUCKS BUT WORKS AS A PLACEHOLDER IG
    public enum Goal {
        STOW,
        HANDOFF,
        L1,
        L2,
        L3,
        L4,
        L4_DUNK,
        L4_PLOP,
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
        // While in this state it is the job of the superstructure to recover the mechanism to a
        // known safe position
        // Failing this, safely disengage
        UNKNOWN,
        // Either a manual override or safety interlock triggered
        // superstructure control is disabled until robot is disabled and re-enabled, or manual
        // re-enablement occurs
        DISABLED
    }

    private Command toStateWristLast(SuperstructureState state) {
        return Commands.sequence(
                wrist.withGoal(state.getElevatorState()), elevator.withGoal(state.getWristState()));
    }

    private Command toStateThroughTransit(SuperstructureState state) {
        return Commands.either(
                toStateSynchronous(state),
                Commands.sequence(
                        wrist.withGoal(WRIST_TRANSIT.getWristState()),
                        elevator.withGoal(state.getElevatorState()),
                        wrist.withGoal(state.getWristState())),
                () -> {
                    double elevatorGoal = state.getElevatorState().position;
                    double elevatorInitial = elevator.getPosition();

                    return elevatorInitial < ELEVATOR_MAX_TO_SKIP_TRANSIT
                            && elevatorGoal < ELEVATOR_MAX_TO_SKIP_TRANSIT;
                });
    }

    private Command toStateWristFirst(SuperstructureState state) {
        return Commands.sequence(
                wrist.withGoal(state.getElevatorState()), elevator.withGoal(state.getWristState()));
    }

    private Command toStateSynchronous(SuperstructureState state) {
        return Commands.parallel(
                elevator.withGoal(state.getElevatorState()), wrist.withGoal(state.getWristState()));
    }

    private int dynamicStep = 0;

    public int dynamicStep() {
        return dynamicStep;
    }

    public Command toGoalThenIdle(Goal goal) {
        return toGoal(goal).andThen(idle());
    }

    public Command toGoal(Goal goal) {
        if (goal == Goal.L4_DUNK) {
            Command command = Commands.none();

            for (StateWithMode stateWithMode : dynamicGoals.get(goal)) {
                command = command.andThen(
                                switch (stateWithMode.mode()) {
                                    case WRIST_FIRST -> toStateWristFirst(stateWithMode.state());
                                    case WRIST_LAST -> toStateWristLast(stateWithMode.state());
                                    case WRIST_SYNC -> toStateSynchronous(stateWithMode.state());
                                })
                        .finallyDo(() -> dynamicStep++);
            }

            command = command.beforeStarting(() -> {
                        this.state = State.IN_TRANSIT;
                        previousGoal = currentGoal;
                        currentGoal = goal;
                        dynamicStep = 0;
                    })
                    .finallyDo((interrupted) -> {
                        this.state = interrupted ? State.UNKNOWN : State.AT_GOAL;
                        dynamicStep = -1;
                    })
                    .deadlineFor(idle());

            return command.onlyIf(() -> currentGoal == Goal.L4)
                    .withName("superstructure/dynamic_" + goal);
        }

        return toStateThroughTransit(staticGoals.get(goal))
                .beforeStarting(() -> {
                    state = State.IN_TRANSIT;
                    previousGoal = currentGoal;
                    currentGoal = goal;
                    dynamicStep = -1;
                })
                .finallyDo((interrupted) -> {
                    state = interrupted ? State.UNKNOWN : State.AT_GOAL;
                })
                .withName("superstructure/static_" + goal)
                .deadlineFor(idle());
    }

    public Command toGoalDirect(Goal goal) {
        return toStateSynchronous(staticGoals.get(goal))
                .beforeStarting(() -> {
                    state = State.IN_TRANSIT;
                    previousGoal = currentGoal;
                    currentGoal = goal;
                    dynamicStep = -1;
                })
                .finallyDo((interrupted) -> {
                    state = interrupted ? State.UNKNOWN : State.AT_GOAL;
                })
                .withName("superstructure/static_direct_" + goal)
                .deadlineFor(idle());
    }

    public boolean atGoal(Goal goal) {
        return currentGoal == goal && state == State.AT_GOAL;
    }

    public Trigger triggerAtGoal(Goal goal) {
        return new Trigger(() -> currentGoal == goal && state == State.AT_GOAL);
    }

    public Command idle() {
        return Commands.idle(this);
    }

    public Command manual(DoubleSupplier elevatorSpeed, DoubleSupplier wristSpeed) {
        return Commands.parallel(
                        Commands.either(
                                        elevator.withSpeed(elevatorSpeed)
                                                .until(() -> elevatorSpeed.getAsDouble() == 0),
                                        Commands.waitSeconds(.1)
                                                .andThen(Commands.defer(
                                                                () -> {
                                                                    double pos =
                                                                            elevator.getPosition();
                                                                    return elevator.followGoal(
                                                                            () -> pos);
                                                                },
                                                                Set.of(elevator))
                                                        .until(() ->
                                                                elevatorSpeed.getAsDouble() != 0)),
                                        () -> elevatorSpeed.getAsDouble() != 0)
                                .repeatedly(),
                        Commands.either(
                                        wrist.setSpeed(wristSpeed)
                                                .until(() -> wristSpeed.getAsDouble() == 0),
                                        Commands.waitSeconds(.1)
                                                .andThen(Commands.defer(
                                                                () -> {
                                                                    double pos =
                                                                            wrist.getPosition();
                                                                    return wrist.followGoal(
                                                                            () -> pos);
                                                                },
                                                                Set.of(wrist))
                                                        .until(() ->
                                                                wristSpeed.getAsDouble() != 0)),
                                        () -> wristSpeed.getAsDouble() != 0)
                                .repeatedly(),
                        this.idle())
                .withName(logKey + "/smart_manual");
    }
}
