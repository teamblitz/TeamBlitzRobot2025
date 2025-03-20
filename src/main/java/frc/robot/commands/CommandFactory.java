package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class CommandFactory {
    public static Command l4Dunk(Superstructure superstructure, Intake intake) {
        return superstructure.toGoal(Superstructure.Goal.L4_DUNK);
    }

    public static Command l4Plop(Superstructure superstructure, Intake intake) {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.L4),
                Commands.waitUntil(() -> superstructure.dynamicStep() == 1)
                        .andThen(intake.setSpeed(Constants.Intake.L4_PLOP))
                        .withTimeout(.25)
        );
    }
}
