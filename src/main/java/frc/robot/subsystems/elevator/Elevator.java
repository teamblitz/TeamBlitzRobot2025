package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;


public class Elevator extends SubsystemBase{
    
    public Elevator() {

    }

    public void periodic() {

    }

    public Command setSpeed(double left, double right) {
        return startEnd (
            () -> {
                io.setSpeedLeft(left);
                io.setSpeedRight(right);
            },
            () -> {
                io.setSpeedLeft(0);
                io.setSpeedRight(0);
            })
            .withName(logKey + "/speed " + left + " " + right);
    }

    public Command l2Extension() {
        return runEnd (
            () -> {
                /* left motor*/(Constants.Elevator.L2_EXTENSION);
                /*right motor*/(Constants.Elevator.L2_EXTENSION);
            }
        )
    }

    public Command l3Extension() {
        return runEnd (
            () -> {
                /*left motor */(Constants.Elevator.L3_EXTENSION);
                /*right motor */(Constants.Elevator.L3_EXTENSION);
            }
        )
    }

    public Command l4Extension() {
        return runEnd (
            () -> {
                /*left motor */(Constants.Elevator.L4_EXTENSION);
                /*right motor */(Constants.Elevator.L4_EXTENSION);
            }
        )
    }
}
