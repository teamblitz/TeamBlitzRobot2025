package frc.robot.commands;

import static frc.robot.Constants.Intake.L4_PLOP;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class CommandFactory {
    public static Command l4Dunk(Superstructure superstructure, Intake intake) {
        return superstructure.toGoalThenIdle(Superstructure.Goal.L4_DUNK);
    }

    public static Command l4Plop(Superstructure superstructure, Intake intake) {
        return Commands.sequence(
                superstructure.toGoalDirect(Superstructure.Goal.L4_PLOP),
                intake.setSpeed(L4_PLOP)
                        .withDeadline(
                                Commands.waitSeconds(0)
                                        .andThen(superstructure.toGoal(Superstructure.Goal.STOW))));
    }

    public static Command handoff(Superstructure superstructure, Intake intake) {
        return superstructure
                .toGoalThenIdle(Superstructure.Goal.HANDOFF)
                .withDeadline(intake.handoff())
                .unless(intake::hasCoral)
                .withName("handoff");
    }
}
