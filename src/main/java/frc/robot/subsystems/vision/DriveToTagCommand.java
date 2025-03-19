package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToTagCommand extends CommandBase {
    private final Vision vision;

    public DriveToTagCommand(Vision vision) {
        this.vision = vision;
        addRequirements(vision);
    }

    public void execute() {
        vision.driveToAprilTags();

    @Override
    public boolean isFinished() {
        return false;
    }

    public void done(boolean interrupt) {
        vision.drive.drive(new Transform2d(0, 0), 0, true);
    }

    }
}
