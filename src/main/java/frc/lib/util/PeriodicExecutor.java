package frc.lib.util;

import frc.robot.Constants;
import lombok.Getter;
import lombok.NonNull;

import java.util.ArrayList;
import java.util.List;

public class PeriodicExecutor {
    /**
     * -- GETTER --
     *  Returns the singleton instance of the PeriodicExecutor.
     */
    @Getter
    private static final PeriodicExecutor instance = new PeriodicExecutor(Constants.LOOP_PERIOD_SEC);

    double basePeriod;
    int loopCount;
    List<PeriodicRunnable> callbacks = new ArrayList<PeriodicRunnable>();

    private PeriodicExecutor(double basePeriod) {
        this.basePeriod = basePeriod;
    }

    /**
     * Schedules a callback to be run periodically with a specified period and optional phase offset.
     *
     * <p>The callback will be executed at most once every {@code periodSeconds}, aligned to the executor's
     * internal base period (typically the robot's main loop period). The offset can be used to stagger
     * execution relative to the loop count, which is useful for load balancing or phasing tasks.</p>
     *
     * @param callback the Runnable to execute periodically (must not be null)
     * @param periodSeconds the interval in seconds between successive executions (must be ≥ base period)
     * @param offsetSeconds the delay in seconds before the first eligible execution (will be aligned to the base period)
     * @throws IllegalArgumentException if {@code periodSeconds} is less than the base execution period
     */
    public void addPeriodicSeconds(@NonNull Runnable callback, double periodSeconds, double offsetSeconds) {
        if (periodSeconds < basePeriod) {
            throw new IllegalArgumentException("Period: " + periodSeconds + " is too short");
        }

        addPeriodicCycles(
                callback,
                (int) Math.max(1, Math.round(periodSeconds / basePeriod)),
                (int) Math.round(offsetSeconds / basePeriod)
        );
    }

    public void addPeriodicCycles(@NonNull Runnable callback, int periodLoops, int offsetLoops) {
        callbacks.add(
                new PeriodicRunnable(
                        callback,
                        periodLoops,
                       offsetLoops
                )
        );
    }

    public void addPeriodicSeconds(Runnable callback, double periodSeconds) {
        addPeriodicSeconds(callback, periodSeconds, 0);
    }

    private record PeriodicRunnable(Runnable callback, int periodLoops, int offsetLoops) {}

    // Should be called ever basePeriod
    public void run() {
        for (PeriodicRunnable periodicRunnable : callbacks) {
            if ((loopCount + periodicRunnable.offsetLoops) % periodicRunnable.periodLoops == 0) {
                periodicRunnable.callback.run();
            }
        }
    }
}
