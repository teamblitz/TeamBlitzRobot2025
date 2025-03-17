/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.StartingPosition;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.range.RangeSensorIO;
import frc.robot.subsystems.drive.range.RangeSensorIOFusion;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSpark;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */
    private Drive drive;
    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;
    private Superstructure superstructure;

    /* ***** --- Autonomous --- ***** */
    private final LoggedDashboardChooser<Command> autoChooser;

    private final LoggedDashboardChooser<StartingPosition> startingPositionChooser;

    public RobotContainer() {
        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();
        configureAutoCommands();

        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("Drive")
                .add("ResetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

        autoChooser = new LoggedDashboardChooser<>("autoChoice", AutoBuilder.buildAutoChooser());

        startingPositionChooser = new LoggedDashboardChooser<>("startingPos");
        startingPositionChooser.addDefaultOption("Center", StartingPosition.CENTER);
        startingPositionChooser.addOption("Left", StartingPosition.LEFT);
        startingPositionChooser.addOption("Right", StartingPosition.RIGHT);
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                new TeleopSwerve(
                                drive,
                                OIConstants.Drive.X_TRANSLATION,
                                OIConstants.Drive.Y_TRANSLATION,
                                OIConstants.Drive.ROTATION_SPEED,
                                () -> false,
                                () -> Double.NaN)
                        .withName("TeleopSwerve"));

        superstructure.setDefaultCommand(
                Commands.either(
                        superstructure.toGoal(Superstructure.Goal.STOW)
                                .onlyWhile(intake::hasCoral),
                        superstructure.toGoal(Superstructure.Goal.HANDOFF)
                                .until(intake::hasCoral),
                        intake::hasCoral
                ).withName("superstructure/conditionalDefault")
        );
    }

    private void configureSubsystems() {
        drive =
                switch (Constants.ROBOT) {
                    case CompBot ->
                            new Drive(
                                    new SwerveModuleConfiguration(
                                            SwerveModuleConfiguration.MotorType.KRAKEN,
                                            SwerveModuleConfiguration.MotorType.NEO,
                                            SwerveModuleConfiguration.EncoderType.CANCODER),
                                    Constants.Drive.Mod0.CONSTANTS,
                                    Constants.Drive.Mod1.CONSTANTS,
                                    Constants.Drive.Mod2.CONSTANTS,
                                    Constants.Drive.Mod3.CONSTANTS,
                                    new GyroIOPigeon(),
                                    new RangeSensorIOFusion());

                    case DevBot ->
                            new Drive(
                                    new SwerveModuleConfiguration(
                                            SwerveModuleConfiguration.MotorType.NEO,
                                            SwerveModuleConfiguration.MotorType.NEO,
                                            SwerveModuleConfiguration.EncoderType.HELIUM),
                                    Constants.Drive.Mod0.CONSTANTS,
                                    Constants.Drive.Mod1.CONSTANTS,
                                    Constants.Drive.Mod2.CONSTANTS,
                                    Constants.Drive.Mod3.CONSTANTS,
                                    new GyroIOPigeon(),
                                    new RangeSensorIO() {});
                    case SimBot ->
                            new Drive(
                                    new SwerveModule(
                                            Constants.Drive.FL,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new SwerveModule(
                                            Constants.Drive.FR,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new SwerveModule(
                                            Constants.Drive.BL,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new SwerveModule(
                                            Constants.Drive.BR,
                                            new AngleMotorIOSim(),
                                            new DriveMotorIOSim(),
                                            new EncoderIO() {}),
                                    new GyroIO() {},
                                    new RangeSensorIO() {});
                };



        //        wrist = new Wrist(new WristIO() {});

        intake = new Intake(new IntakeIOSpark());
        //        intake = new Intake(new IntakeIO() {});

        superstructure = new Superstructure();

        elevator = superstructure.getElevator();
        wrist = superstructure.getWrist();
    }

    private void configureButtonBindings() {
        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(drive::zeroGyro));
        OIConstants.Drive.X_BREAK.onTrue(drive.park());

        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        OIConstants.SuperStructure.L1.whileTrue(superstructure.toGoal(Superstructure.Goal.L1));
        OIConstants.SuperStructure.L2.whileTrue(superstructure.toGoal(Superstructure.Goal.L2));
        OIConstants.SuperStructure.L3.whileTrue(superstructure.toGoal(Superstructure.Goal.L3));
        OIConstants.SuperStructure.L4.whileTrue(superstructure.toGoal(Superstructure.Goal.L4));

        OIConstants.SuperStructure.KICK_BOTTOM_ALGAE.whileTrue(
                superstructure
                        .toGoal(Superstructure.Goal.KICK_LOW_ALGAE)
                        .alongWith(intake.kick_algae()));
        OIConstants.SuperStructure.KICK_TOP_ALGAE.whileTrue(
                superstructure
                        .toGoal(Superstructure.Goal.KICK_HIGH_ALGAE)
                        .alongWith(intake.kick_algae()));

        OIConstants.SuperStructure.HANDOFF.whileTrue(
                superstructure.toGoal(Superstructure.Goal.HANDOFF).withDeadline(intake.handoff()).unless(intake::hasCoral).withName("handoff"));

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
                .onTrue(CommandFactory.l4Dunk(superstructure, intake));

        OIConstants.Intake.HANDOFF.whileTrue(intake.handoff());
        OIConstants.Intake.REVERSE.whileTrue(intake.reverse());
        OIConstants.Intake.ALGAE_REMOVAL.whileTrue(intake.kick_algae());
        OIConstants.Intake.SHOOT_CORAL.whileTrue(intake.shoot_coral());

        OIConstants.Elevator.MANUAL_UP.whileTrue(
                elevator.upManual().alongWith(superstructure.idle()).withName("elevator/manual_up"));
        OIConstants.Elevator.MANUAL_DOWN.whileTrue(
                elevator.downManual().alongWith(superstructure.idle()).withName("elevator/manual_down"));

        new Trigger(() -> Math.abs(OIConstants.Wrist.WRIST_MANUAL.getAsDouble()) > .07)
                .whileTrue(
                        wrist.setSpeed(OIConstants.Wrist.WRIST_MANUAL)
                                .alongWith(superstructure.idle()));

        new Trigger(RobotController::getUserButton)
                .toggleOnTrue(
                        Commands.parallel(wrist.coastCommand(), elevator.coastCommand())
                                .onlyWhile(RobotState::isDisabled));
    }

    private void configureAutoCommands() {}

    public Command getAutonomousCommand() { // Autonomous code goes here
        return Commands.runOnce(() -> drive.setGyro(startingPositionChooser.get().angle))
                .andThen(autoChooser.get().asProxy());
    }
}
