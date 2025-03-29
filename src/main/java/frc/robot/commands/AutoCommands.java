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
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;


public class AutoCommands {
    private final Drive drive;
    private final SwerveDriveKinematics kinematics;
    private final AutoFactory autoFactory;
    private final Superstructure superstructure;
    private final Intake intake;

    public AutoCommands(Drive drive, Superstructure superstructure, Intake intake) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;
        this.kinematics = Constants.Drive.KINEMATICS;


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

        routine.active().whileTrue(
                Commands.sequence(
                        traj.resetOdometry(),
                        traj.cmd()
                ).withName("auto/cmdSec")
        );


        return routine.cmd().withName("auto/test");
    }

    public AutoRoutine twoPiece() {
        AutoRoutine routine = autoFactory.newRoutine("twoPiece");

        AutoTrajectory scoreInitial = routine.trajectory("twoPiece", 0);
        AutoTrajectory toStation = routine.trajectory("twoPiece",1);
        AutoTrajectory scoreSecond = routine.trajectory("twoPiece", 2);
        AutoTrajectory gtfo = routine.trajectory("twoPiece", 3);

        scoreSecond.atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY).onTrue(prepareL4());

        routine.active().onTrue(
                Commands.sequence(
                        scoreInitial.resetOdometry(),
                        scoreInitial.cmd()
                )
        );

        scoreInitial.atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY).onTrue(prepareL4());
        scoreInitial.done().onTrue(
                scoreL4().andThen(toStation.cmd()
                )
        );


        toStation.active().onTrue(handoff());

        toStation.chain(scoreSecond);


        scoreSecond.atTimeBeforeEnd(Constants.Auto.Timings.STOW_TO_L4_READY)
                        .onTrue(
                                Commands.sequence(
//                                        Commands.waitUntil(intake::hasCoral),
                                        prepareL4()
                                ));

        scoreSecond.done().onTrue(
                Commands.sequence(
                        Commands.waitUntil(superstructure.triggerAtGoal(Superstructure.Goal.L4)),
                        scoreL4(),
                        gtfo.cmd()
                )
        );



        return routine;

    }
//
//    public AutoRoutine fourPiece() {
//        AutoRoutine routine = autoFactory.newRoutine("fourPiece");
//
//        AutoTrajectory pickup = routine.trajectory("pickup", 0);
//    }

    private Command scoreL4() {
        return Commands.sequence(
                superstructure.toGoal(Superstructure.Goal.L4),
                CommandFactory.l4Plop(superstructure, intake)
        );
    }

    private Command prepareL4() {
        return superstructure.toGoalThenIdle(Superstructure.Goal.L4);
    }

    private Command handoff() {
        return CommandFactory.handoff(superstructure, intake);
    }

}
