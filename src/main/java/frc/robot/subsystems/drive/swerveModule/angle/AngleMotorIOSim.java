package frc.robot.subsystems.drive.swerveModule.angle;

public class AngleMotorIOSim implements AngleMotorIO {
    private double rotation;

    @Override
    public void updateInputs(AngleMotorIO.AngleMotorInputs inputs) {
        inputs.rotation = rotation;
    }

    @Override
    public void setSetpoint(double setpoint) {
        rotation = setpoint;
    }
}
