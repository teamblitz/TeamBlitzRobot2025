package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.winch.Winch;

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

    public static Command readyClimb(Climber climber, Winch winch) {
        return winch.raiseFunnel().andThen(climber.deployClimber());
    }

    public static Command restoreClimber(Climber climber, Winch winch) {
        return winch.raiseFunnel().andThen(climber.restowClimber()).andThen(winch.lowerFunnel());
    }

    public static Command handoff(Superstructure superstructure, Intake intake) {
        return superstructure
                .toGoalThenIdle(Superstructure.Goal.HANDOFF)
                .withDeadline(intake.handoff())
                .unless(intake::hasCoral)
                .withName("handoff");
    }

}
