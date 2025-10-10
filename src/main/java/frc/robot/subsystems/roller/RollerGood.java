package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Roller.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RollerGood extends SubsystemBase {
    TalonFX roller;

    public RollerGood() {
        roller = new TalonFX(ROLLER_ID);
    }

    public Command score() {
         return startEnd(
            () -> {
                roller.set(1);
            },
            () -> {
                roller.set(0);
            });
            /*return run(
                () -> {
                roller.set(1);
                });*/
             
    }

}
