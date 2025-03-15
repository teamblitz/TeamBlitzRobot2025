package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

public class CommandFactory {
    public static Command l4Dunk(Superstructure superstructure, Intake intake) {
        return superstructure
                .toGoal(Superstructure.Goal.L4_DUNK);
    }
}
