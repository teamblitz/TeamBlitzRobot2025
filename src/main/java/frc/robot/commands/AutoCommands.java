package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public class AutoCommands {
    private final Drive drive;
    private final SwerveDriveKinematics kinematics;
    private final AutoFactory autoFactory;

    private final PIDController x = new PIDController(0, 0, 0);
    private final PIDController y = new PIDController(0, 0, 0);
    private final PIDController theta = new PIDController(0, 0, 0);

    public AutoCommands(Drive drive) {
        this.drive = drive;
        this.kinematics = Constants.Drive.KINEMATICS;
        theta.enableContinuousInput(-Math.PI, Math.PI);

        var trajectory = Choreo.loadTrajectory("testDrive");

        autoFactory = new AutoFactory(
            drive::getPose,
            drive::resetPose,
            drive::followTrajectory,
            true,
            drive
        );
    }

    public AutoFactory getFactory() {
        return autoFactory;
    }

    public Command getNoAuto() {
        final var routine = autoFactory.newRoutine("None");
        routine.active().onTrue(Commands.print("Running No Auto"));
        return routine.cmd();
    }

    public Command testDrive() {
        final var routine = autoFactory.newRoutine("test");
        final var traj = routine.trajectory("test");

        routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
        return routine.cmd();
    }

}
