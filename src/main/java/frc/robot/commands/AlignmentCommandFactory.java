package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.PositionConstants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import java.util.Set;

public class AlignmentCommandFactory {

    public static Command alignLeftReefPole(CommandSwerveDrivetrain drive, DriveCommands drivecommands) {
        return new DeferredCommand(
                () -> {
                    Pose2d pose = PositionConstants.Reef.SCORING_POSITIONS.get(
                            PositionConstants.getClosestFace(drive.getPose())[0]).get();
                    return alignToPose(
                            pose,
                            drive,
                            drivecommands
                    );
                }, Set.of(drive));
    }

    public static Command alignRightReefPole(CommandSwerveDrivetrain drive, DriveCommands drivecommands) {
        return new DeferredCommand(
                () -> {
                    Pose2d pose = PositionConstants.Reef.SCORING_POSITIONS.get(
                            PositionConstants.getClosestFace(drive.getPose())[1]).get();
                    return alignToPose(
                            pose,
                            drive,
                            drivecommands
                    );
                }, Set.of(drive));
    }

    private static Command alignToPose(Pose2d pose, CommandSwerveDrivetrain drive, DriveCommands drivecommands) {
        return drivecommands.pullToPose(
                () -> pose,
                6,
                2,
                3,
                .02,
                .5 * Math.PI,
                Math.PI
        );
    }

    private AlignmentCommandFactory() {}
}
