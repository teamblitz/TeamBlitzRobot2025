package frc.robot.subsystems.roller;

public interface RollerIO {
    default void setSpeed(double speed) {}

    public static double ROLLER_SPEED = 0.3;
}
