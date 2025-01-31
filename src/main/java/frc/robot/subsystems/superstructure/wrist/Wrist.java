package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.wrist.WristIOSpark;
import frc.robot.subsystems.superstructure.wrist.WristIO.WristIOInputs;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOKraken;


public class Wrist extends BlitzSubsystem {
    public final frc.robot.subsystems.superstructure.wrist.WristIO io;

    public Wrist(WristIO io) {
        super("wrist");
        this.io = io;
    }

    @Override
    public void periodic() {
    }
     public Command setSpeed(double wristMotor) {
         return startEnd(
                 () -> {
                     io.setSpeedwristMotor(0);
                 },
                 () -> {
                     io.setSpeedwristMotor(0);
                 })
                 .withName(logKey + "/speed " + wristMotor);
     }

     public Command r1Rotation() {
         return runEnd(
                 () -> {
                     io.setSetpoint(WristIO.wristRotations.r1RotationValue, 0, 0);
                 }
         );
     }

     public Command r2Rotation() {
         return Commands.none();
         return runEnd(
                 () -> {
                     wristMotor.set(WristIO.wristRotations.r2RotationValue);
                 }
         );
     }

     public Command r3Rotation() {
         return Commands.none();
         return runEnd(
                 () -> {
                     wristMotor.set(WristIO.wristRotations.r3RotationValue);
                 }
         );
     }
 }
