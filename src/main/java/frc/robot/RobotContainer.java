/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.math.AllianceFlipUtil;
import frc.lib.reefscape.ScoringPositions;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSpark;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIOKraken;
import frc.robot.subsystems.superstructure.wrist.WristIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.winch.Winch;
import frc.robot.subsystems.winch.WinchIO;
import frc.robot.subsystems.winch.WinchIOSpark;

import org.littletonrobotics.junction.Logger;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */
    private CommandSwerveDrivetrain drive;
    private Vision vision;
    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;
    private Superstructure superstructure;
    private Winch winch;
    private Climber climber;
    private AutoCommands autoCommands;
    private DriveCommands driveCommands;

    /* ***** --- Autonomous --- ***** */
    private AutoChooser autoChooser;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        // Configure Subsystems
        configureSubsystems();
        // Set default commands
        setDefaultCommands();
        // Configure Trigger Bindings
        configureTriggerBindings();
        // Configure Autonomous
        configureAutonomous();

        configureDashboard();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureSubsystems() {
        drive = TunerConstants.createDrivetrain();
        driveCommands = new DriveCommands(drive);

        vision = new Vision(drive);

        intake = new Intake(Constants.compBot() ? new IntakeIOKraken() : new IntakeIOSpark());

        superstructure = new Superstructure(
                Constants.compBot() ? new ElevatorIOKraken() : new ElevatorIOSpark(),
                Constants.compBot() ? new WristIOKraken() : new WristIOSpark());
        elevator = superstructure.getElevator();
        wrist = superstructure.getWrist();

        climber = new Climber(Constants.compBot() ? new ClimberIOKraken() : new ClimberIO() {});
        winch = new Winch(Constants.compBot() ? new WinchIOSpark() : new WinchIO() {});
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(driveCommands
                .joystickDrive(
                        OIConstants.Drive.X_TRANSLATION,
                        OIConstants.Drive.Y_TRANSLATION,
                        OIConstants.Drive.ROTATION_SPEED,
                        () -> 5,
                        () -> 10,
                        () -> 2 * Math.PI,
                        true)
                .onlyWhile(RobotState::isTeleop)
                .onlyIf(RobotState::isTeleop)
                .withName("Joystick Drive"));

        superstructure.setDefaultCommand(Commands.either(
                        superstructure
                                .toGoalThenIdle(Superstructure.Goal.STOW)
                                .onlyWhile(intake::hasCoral),
                        superstructure
                                .toGoalThenIdle(Superstructure.Goal.HANDOFF)
                                .until(intake::hasCoral),
                        intake::hasCoral)
                .withName("superstructure/conditionalDefault"));
    }

    private void configureTriggerBindings() {
        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(() -> drive.resetRotation(
                AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero)));
        //        OIConstants.Drive.X_BREAK.onTrue(drive.park());
        //
        //        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        //        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        OIConstants.SuperStructure.L1.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L1));
        OIConstants.SuperStructure.L2.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L2));
        OIConstants.SuperStructure.L3.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L3));
        OIConstants.SuperStructure.L4.whileTrue(
                superstructure.toGoalThenIdle(Superstructure.Goal.L4));

        OIConstants.SuperStructure.KICK_BOTTOM_ALGAE.whileTrue(superstructure
                .toGoalThenIdle(Superstructure.Goal.KICK_LOW_ALGAE)
                .alongWith(intake.kick_algae()));
        OIConstants.SuperStructure.KICK_TOP_ALGAE.whileTrue(superstructure
                .toGoalThenIdle(Superstructure.Goal.KICK_HIGH_ALGAE)
                .alongWith(intake.kick_algae()));

        OIConstants.SuperStructure.HANDOFF.whileTrue(
                CommandFactory.handoff(superstructure, intake));

        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L1))
                .whileTrue(intake.coral_l1());
        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L2))
                .whileTrue(intake.shoot_coral());
        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L3))
                .whileTrue(intake.shoot_coral());
        OIConstants.SuperStructure.SCORE
                .and(superstructure.triggerAtGoal(Superstructure.Goal.L4))
                .onTrue(CommandFactory.l4Plop(superstructure, intake));

        OIConstants.Intake.HANDOFF.whileTrue(intake.handoff());
        OIConstants.Intake.REVERSE.whileTrue(intake.reverse());
        OIConstants.Intake.ALGAE_REMOVAL.whileTrue(intake.kick_algae());
        OIConstants.Intake.SHOOT_CORAL.whileTrue(intake.shoot_coral());

        OIConstants.Intake.INTAKE_ALGAE.whileTrue(intake.setSpeed(Constants.Intake.ALGAE_HOLD));
        OIConstants.Intake.EJECT_ALGAE.whileTrue(intake.setSpeed(Constants.Intake.ALGAE_REMOVAL));

        OIConstants.Winch.WINCH_MAN_UP.whileTrue(winch.manualUp());
        OIConstants.Winch.WINCH_MAN_DOWN.whileTrue(winch.manualDown());

        OIConstants.SuperStructure.MANUAL_MODE.onTrue(
                superstructure.manual(OIConstants.Elevator.MANUAL, OIConstants.Wrist.MANUAL));

        OIConstants.Climber.CLIMBER_UP_MAN.whileTrue(climber.setSpeed(.8));
        OIConstants.Climber.CLIMBER_DOWN_MAN.whileTrue(climber.setSpeed(-.8));

        OIConstants.Climber.DEPLOY_CLIMBER.onTrue(
                ClimbCommandFactory.deployClimber(climber, winch));
        OIConstants.Climber.RESTOW_CLIMBER.onTrue(ClimbCommandFactory.stowClimber(climber, winch)
                .unless(() -> climber.getState() == Climber.State.CLIMB));

        OIConstants.SuperStructure.SCORE
                .and(() -> climber.getState() == Climber.State.DEPLOYED
                        || climber.getState() == Climber.State.CLIMB)
                .whileTrue(climber.climb());

        new Trigger(RobotController::getUserButton)
                .toggleOnTrue(Commands.parallel(
                                wrist.coastCommand(),
                                elevator.coastCommand(),
                                climber.coastCommand())
                        .onlyWhile(RobotState::isDisabled));

        OIConstants.Drive.ALIGN_LEFT.whileTrue(new DeferredCommand(
                () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
                        PositionConstants.getClosestFace(drive.getPose())[0])),
                Set.of(drive)));

        OIConstants.Drive.ALIGN_RIGHT.whileTrue(new DeferredCommand(
                () -> drive.driveToPose(PositionConstants.Reef.SCORING_POSITIONS.get(
                        PositionConstants.getClosestFace(drive.getPose())[1])),
                Set.of(drive)));
    }

    private void configureDashboard() {
//        var tab = Shuffleboard.getTab("tuning");
//
//        tab.add(
//                "Phoenix SignalLogger",
//                runEnd(SignalLogger::start, SignalLogger::stop).ignoringDisable(true));
//
//        tab.add("drive/resetOdometry", Commands.runOnce(() -> drive.resetPose(new Pose2d())));
//
//        tab.add(
//                "wheel radius characterization",
//                DriveCharacterizationCommands.characterizeWheelDiameter(drive));

        new Trigger(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Blue)
                .onChange(runOnce(() -> {
                            for (ScoringPositions.Branch branch :
                                    ScoringPositions.Branch.values()) {
                                Logger.recordOutput(
                                        "positions/reef/" + branch.name(),
                                        PositionConstants.Reef.SCORING_POSITIONS
                                                .get(branch)
                                                .get());
                            }
                        })
                        .ignoringDisable(true));

        Commands.run(() -> {
                    PositionConstants.getClosestFace(drive.getPose());
                })
                .ignoringDisable(true)
                .onlyIf(Robot::isSimulation)
                .schedule();
    }

    private void configureAutonomous() {
        autoChooser = new AutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

        autoCommands = new AutoCommands(drive, vision, superstructure, intake, driveCommands);

        autoChooser.addRoutine("twoPiece", autoCommands::twoPiece);
        //        autoChooser.addRoutine("test", autoCommands::testDrive);
        autoChooser.addRoutine("fourPieceLeft", autoCommands::fourPieceLeft);
        autoChooser.addRoutine("leaveRight", () -> autoCommands.leave("leaveRight"));

        autoChooser.addRoutine("demoFollowAprilTag", () -> autoCommands.demoFollowAprilTag());
    }

    public Command getAutonomousCommand() {
        Logger.recordOutput("selectedAuto", autoChooser.selectedCommand().getName());
        return Commands.sequence(
                        Commands.runOnce(() -> drive.resetRotation(
                                AllianceFlipUtil.shouldFlip()
                                        ? Rotation2d.kZero
                                        : Rotation2d.k180deg)),
                        autoChooser.selectedCommandScheduler())
                .withName("Auto Command");
    }
}
