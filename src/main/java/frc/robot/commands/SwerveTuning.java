package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SwerveTuning extends Command {
    private final Drive drive;

    private final ShuffleboardTab tab;

    private double angle;
    private double speed;

    public SwerveTuning(Drive drive) {
        this.drive = drive;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drive);

        this.tab = Shuffleboard.getTab("DriveTuning");
    }

    @Override
    public void initialize() {
        System.out.println("Drive tuning init");
    }

    @Override
    public void execute() {
        // if (DriverStation.isTest()) {
        drive.setModuleStates(
                new SwerveModuleState[] {
                    new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)),
                    new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)),
                    new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)),
                    new SwerveModuleState(speed, Rotation2d.fromDegrees(angle))
                },
                true,
                true,
                false);
        // }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false, true, false);
        System.out.println("swervetuningend");
    }

    public void nextAngle() {
        angle += 90;
        angle %= 360;
        System.out.println("NextAngle");
    }
}
