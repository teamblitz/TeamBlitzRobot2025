package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Capture;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import java.util.Arrays;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

// TODO: Merge with DriveSysid
public class DriveCharacterizationCommands {

    private static final SwerveRequest.SysIdSwerveRotation wheelDiameterRequest = new SwerveRequest.SysIdSwerveRotation();

    private static final double WHEEL_DIAMETER_TEST_SPEED = 1;
    private static final double WHEEL_DIAMETER_TEST_DURATION = 10;

    public static Command characterizeWheelDiameter(CommandSwerveDrivetrain drive) {
        int numModules = drive.getModules().length;

        Capture<Angle> initialYaw = new Capture<>(null);
        Capture<SwerveModulePosition[]> initialPositions = new Capture<>(null);

        SlewRateLimiter slewRateLimiter = new SlewRateLimiter(.5);
        Timer timer = new Timer();

        return Commands.sequence(
                drive.applyRequest(() -> wheelDiameterRequest.withRotationalRate(0)).withTimeout(1),
                Commands.runOnce(
                        () -> {
                            slewRateLimiter.reset(0);
                            timer.restart();

                            initialYaw.inner = drive.getPigeon2().getYaw().getValue();
                            initialPositions.inner = drive.getState().clone().ModulePositions;
                        }
                ),
                drive.applyRequest(
                        () -> wheelDiameterRequest.withRotationalRate(slewRateLimiter.calculate(WHEEL_DIAMETER_TEST_SPEED))
                ).withTimeout(WHEEL_DIAMETER_TEST_DURATION),
                drive.applyRequest(
                        () -> wheelDiameterRequest.withRotationalRate(slewRateLimiter.calculate(0))
                ).until(() -> slewRateLimiter.lastValue() == 0),
                Commands.waitSeconds(.5),
                Commands.runOnce(
                        () -> {
                            timer.stop();

                            Angle finalYaw = drive.getPigeon2().getYaw().getValue();
                            SwerveModulePosition[] finalPositions = drive.getState().ModulePositions;

                            var assumedWheelRadius = TunerConstants.FrontLeft.WheelRadius;

                            // Abs value so we don't get negative wheel diameter
                            var yawDeltaRadians = Math.abs(finalYaw.minus(initialYaw.get()).in(Radians));

                            double[] totalRotations = IntStream.range(0, numModules).mapToDouble(
                                    i -> {
                                        // same here
                                        var metersTraveled = Math.abs(finalPositions[i].distanceMeters - initialPositions.get()[i].distanceMeters);

                                        return metersTraveled / (2 * assumedWheelRadius * Math.PI);
                                    }).toArray();

                            System.out.printf("Wheel Diameter Characterization Completed in %.2f seconds\n", timer.get());
                            System.out.printf("Yaw delta: %.2f\n", Math.toDegrees(yawDeltaRadians));

                            double[] moduleDiametersInches = new double[numModules];

                            for (int i = 0; i < numModules; i++) {
                                var moduleDistMeters = drive.getModuleLocations()[i].getNorm() * yawDeltaRadians;

                                // dist traveled / rotations = dist per rotation (circumference)
                                // circumference / pi = diameter
                                var moduleDiameter = (moduleDistMeters / totalRotations[i]) / Math.PI;

                                var moduleDiameterInches = Units.metersToInches(moduleDiameter);
                                moduleDiametersInches[i] = moduleDiameterInches;

                                System.out.printf("Module %d rotated %.2f times. Calculated diameter: %.10f in\n", i, totalRotations[i], moduleDiameterInches);
                            }

                            var averageDiameter = Arrays.stream(moduleDiametersInches).summaryStatistics().getAverage();
                            var diameterStdDev = new StandardDeviation().evaluate(moduleDiametersInches);

                            System.out.printf("Average diameter: %.10f\n", averageDiameter);
                            System.out.printf("Standard Deviation: %.4f\n", diameterStdDev);
                        }
                )

        );

    }




    private DriveCharacterizationCommands() {}
}
