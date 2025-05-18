package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.winch.Winch;

public class ClimbCommandFactory {
    public static Command deployClimber(Climber climber, Winch winch) {
        return winch.raiseFunnel().andThen(climber.deployClimber());
    }

    public static Command stowClimber(Climber climber, Winch winch) {
        return winch.raiseFunnel().andThen(climber.restowClimber()).andThen(winch.lowerFunnel());
    }

    public static Command climb(Climber climber, Winch winch) {
        return winch.raiseFunnel().andThen(climber.climb());
    }
}
