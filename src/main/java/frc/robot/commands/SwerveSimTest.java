package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SwerveSimTest extends Command {
    private final Drive drive;
    private final GenericEntry translationEntry;
    private final GenericEntry strafeEntry;
    private final GenericEntry rotationEntry;

    public SwerveSimTest(Drive s_Swerve) {
        this.drive = s_Swerve;
        addRequirements(s_Swerve);

        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        translationEntry = tab.add("Translation", 0).getEntry();
        strafeEntry = tab.add("Strafe", 0).getEntry();
        rotationEntry = tab.add("Rotation", 0).getEntry();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drive.drive(
                new Translation2d(translationEntry.getDouble(0), strafeEntry.getDouble(0)),
                rotationEntry.getDouble(0),
                false,
                true,
                true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
