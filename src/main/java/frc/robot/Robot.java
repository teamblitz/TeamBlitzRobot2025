/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.leds.Leds;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    @SuppressWarnings("all") // Supress switch warnings
    public void robotInit() {
        System.out.println("Robot Start up at: " + Timer.getFPGATimestamp());

        Leds.getInstance(); // Start leds

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source

        // Running on a real robot, try to log to a USB stick, else log to the roboRIO
        if (isReal()) {
            String logDir = Filesystem.getOperatingDirectory().getAbsolutePath();
            boolean thumbDriveConnected;
            try {
                // The /u directory maps to the connected usb drive if one exists.
                // If /u doesn't work replace it with /media/sda1
                Path usbDir = Paths.get("/media/sda1").toRealPath();
                thumbDriveConnected = Files.isWritable(usbDir);
                if (thumbDriveConnected) {
                    logDir = usbDir.toString();
                    System.out.println("USB drive connected, will log to USB drive.");
                } else {
                    System.out.printf(
                            "USB drive not found, will log to the %s directory on the roboRIO.%n",
                            logDir);
                }
            } catch (IOException e) {
                System.out.printf(
                        "An IOException occurred when checking for a USB drive, will try to log to %s directory on the roboRIO.%n",
                        logDir);
            }

            Logger.addDataReceiver(new WPILOGWriter(logDir));
            Logger.addDataReceiver(new NT4Publisher());

        } else
            switch (Constants.SIM_MODE) {
                    // Running a physics simulator, log to local folder
                case SIM -> {
                    Logger.addDataReceiver(new WPILOGWriter(""));
                    Logger.addDataReceiver(new NT4Publisher());
                }
                    // Replaying a log, set up replay source
                case REPLAY -> {
                    setUseTiming(false); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog();
                    Logger.setReplaySource(new WPILOGReader(logPath));
                    Logger.addDataReceiver(
                            new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                }
            }

        // Start AdvantageKit logger
        Logger.registerURCL(URCL.startExternal());
        Logger.start();

        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
                (Command command, Boolean active) -> {
                    String name = command.getName();
                    int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
                    commandCounts.put(name, count);
                    Logger.recordOutput(
                            "CommandsUnique/"
                                    + name
                                    + "_"
                                    + Integer.toHexString(command.hashCode()),
                            active);
                    Logger.recordOutput("CommandsAll/" + name, count > 0);
                };
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        (Command command) -> {
                            logCommandFunction.accept(command, true);
                        });
        CommandScheduler.getInstance()
                .onCommandFinish(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                        });
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                        });

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /* ***** --- Autonomous --- ***** */

    // Called at the start of autonomous.
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule autonomous commands
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    // Called periodically during autonomous
    @Override
    public void autonomousPeriodic() {}

    // Called at the end of autonomous
    @Override
    public void autonomousExit() {
        // Cancel autonomous commands
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /* ***** --- Teleop --- ***** */

    // Called at the start of teleop
    @Override
    public void teleopInit() {
        System.out.println("TeleopInit");
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /* ***** --- Test Mode --- ***** */

    // Called at the start of test mode
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /* ***** --- Simulation --- ***** */

    // Called when the robot enters simulation
    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }
}
