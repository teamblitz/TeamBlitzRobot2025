package frc.robot.utils;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Utils {
    /**
     * Runs the command scheduler at the specified rate in milliseconds for the specified amount of
     * iterations. Assumes mock time is already enabled
     *
     * @pram rate The mock rate that the schedular will be ran at.
     * @pram iterations How many iterations to run.
     */
    public static void mockRunScheduler(long rate, long iterations) {
        WPIUtilJNI.setMockTime(WPIUtilJNI.now());
        for (long i = 0; i < iterations; i++) {
            WPIUtilJNI.setMockTime(WPIUtilJNI.now() + rate * 1000L);
            CommandScheduler.getInstance().run();
        }
    }

    /**
     * Runs the command scheduler at the default rate for the specified amount of iterations
     *
     * @pram iterations How many iterations to run.
     */
    public static void mockRunScheduler(long iterations) {
        mockRunScheduler(20, iterations);
    }
}
