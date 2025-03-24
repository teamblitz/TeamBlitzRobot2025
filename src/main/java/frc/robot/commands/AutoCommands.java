package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drive.Drive;
import choreo.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public class AutoCommands {
    private final Drive drive;
    private final SwerveDriveKinematics kinematics;

    private final PIDController x = new PIDController(0, 0, 0);
    private final PIDController y = new PIDController(0, 0, 0);
    private final PIDController theta = new PIDController(0, 0, 0);

    public AutoCommands(Drive drive, SwerveDriveKinematics kinematics) {
        this.drive = drive;
        this.kinematics = kinematics;
        theta.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command defaultDriveAuto() {
       /*  ChoreoTrajectory trajectory = Choreo.getTrajectory(); 

        return new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            kinematics,
            x,
            y,
            theta,
            drive::setModuleStates,
            drive
            ).beforeStarting(() -> {
                Pose2d intialPose = trajectory.getInitialPose();
                drive.resetOdometry(intialPose);
        });
*/

        

    }

    
    
}
