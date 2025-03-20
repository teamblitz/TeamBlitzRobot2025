package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

import static frc.robot.Constants.Intake.L4_PLOP;

public class CommandFactory {
    public static Command l4Dunk(Superstructure superstructure, Intake intake) {
        return superstructure.toGoalThenIdle(Superstructure.Goal.L4_DUNK);
    }

    public static Command l4Plop(Superstructure superstructure, Intake intake) {
        return Commands.sequence(
                superstructure.toGoal(Superstructure.Goal.L4_PLOP),
                intake.setSpeed(L4_PLOP).withDeadline(
                        Commands.waitSeconds(.5).andThen(
                        superstructure.toGoal(Superstructure.Goal.L4))
                )
        );
    }
}
