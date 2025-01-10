package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.math.Trajectories;
import frc.lib.util.AllianceUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("resource")
public class AutoAimCalculator {
    private static final ShuffleboardTab tab = Shuffleboard.getTab("AutoShoot");

    private static final GenericEntry angleOffset = tab.addPersistent("angleOffset", 0).getEntry();
    private static final GenericEntry vertOffset = tab.addPersistent("vertOffset", 0).getEntry();

    private AutoAimCalculator() {}

    /**
     * Puts the bot and the goal into bot goal space with the bot as the origin and the goal as a
     * point along a vertical plane
     */
    public static Pose2d calculateBotToGoal2d(Pose3d botPose, Pose3d goalPose) {
        double horizontalDist =
                Math.hypot(botPose.getX() - goalPose.getX(), botPose.getY() - goalPose.getY());
        Logger.recordOutput("AutoShoot/botPose", botPose);
        Logger.recordOutput("AutoShoot/goalPose", goalPose);

        Logger.recordOutput("AutoShoot/horizontalDist", horizontalDist);

        double vertDist =
                goalPose.getZ() - botPose.getZ() + Units.inchesToMeters(vertOffset.getDouble(0));
        Logger.recordOutput("AutoShoot/vertDist", vertDist);

        return new Pose2d(horizontalDist, vertDist, Rotation2d.fromRadians(0));
    }

    public static Transform2d calculateCenterRotationToShooter(
            Transform2d centerRotationToShooterAtZero, Rotation2d rotation) {
        return new Transform2d(
                centerRotationToShooterAtZero.getTranslation().rotateBy(rotation),
                Rotation2d.fromRadians(0));
    }

    public static double calculateArmAngle(
            Pose3d botPose,
            Pose3d goalPose,
            Transform2d botToCenterRot,
            Transform2d centerRotationToShooterAtZero,
            double shooterAngleOffset,
            double initialVelocity) {
        Pose2d goalPose2d = calculateBotToGoal2d(botPose, goalPose);
        Logger.recordOutput("AutoShoot/goalPose2d", goalPose2d);

        double armRot = Units.degreesToRadians(45); // Start at 45 degrees

        Pose2d shooterPoseDebug = null;
        Translation2d shooterToGoalDebug = null;
        double shooterAngleDebug = 0;

        for (int i = 0; i < 10; i++) {
            Pose2d shooterPose =
                    new Pose2d()
                            .transformBy(botToCenterRot)
                            .transformBy(
                                    calculateCenterRotationToShooter(
                                            centerRotationToShooterAtZero,
                                            Rotation2d.fromRadians(armRot)));

            shooterPoseDebug = shooterPose;

            Translation2d shooterToGoal = new Transform2d(shooterPose, goalPose2d).getTranslation();

            shooterToGoalDebug = shooterToGoal;

            double shooterAngle =
                    Trajectories.angleRequiredToHitCoordinate(
                            shooterToGoal.getX(), shooterToGoal.getY(), initialVelocity, -9.81);
            shooterAngleDebug = shooterAngle;

            armRot = Math.PI / 2 - (shooterAngle + shooterAngleOffset);
        }

        Logger.recordOutput("AutoShoot/shooterToGoal", shooterToGoalDebug);
        Logger.recordOutput("AutoShoot/shooterPose", shooterPoseDebug);
        Logger.recordOutput("AutoShoot/shooterAngle", Units.radiansToDegrees(shooterAngleDebug));

        return armRot + Units.degreesToRadians(angleOffset.getDouble(0));
    }

    public static double calculateArmAngleInterpolation(double distanceMeters) {
        return Constants.Shooter.AutoShootConstants.ANGLE_TREE_MAP.get(distanceMeters)
                + Units.degreesToRadians(1);
    }

    public static double calculateShooterSpeedInterpolation(double distanceMeters) {
        return Constants.Shooter.AutoShootConstants.SHOOT_VELOCITY_TREE_MAP.get(distanceMeters);
    }

    public static double calculateDistanceToGoal(Pose3d botPose) {
        return calculateBotToGoal2d(
                        botPose,
                        DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get()
                                                == DriverStation.Alliance.Blue
                                ? Constants.Shooter.AutoShootConstants.GOAL_POSE_BLUE
                                : Constants.Shooter.AutoShootConstants.GOAL_POSE_RED)
                .getX();
    }

    @AutoLogOutput
    public static Rotation2d calculateSpeakerHeading(Pose2d botPose) {
        Pose2d goalPose =
                (AllianceUtils.isBlue()
                                ? Constants.Shooter.AutoShootConstants.GOAL_POSE_BLUE
                                : Constants.Shooter.AutoShootConstants.GOAL_POSE_RED)
                        .toPose2d();

        Translation2d translation =
                new Transform2d(new Pose2d(botPose.getTranslation(), new Rotation2d()), goalPose)
                        .getTranslation();
        return Rotation2d.fromRadians(-Math.atan2(translation.getY(), translation.getX()))
                .plus(Rotation2d.fromDegrees((AllianceUtils.isBlue() ? 180 : 0) - 10));
    }
}
