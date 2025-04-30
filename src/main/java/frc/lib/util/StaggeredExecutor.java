package frc.lib.util;

import lombok.Getter;
import lombok.NonNull;

import java.util.ArrayList;
import java.util.List;

public class StaggeredExecutor {
    @Getter
    public static final StaggeredExecutor instance = new StaggeredExecutor();


    private List<StaggeredRunnable> toSchedule = new ArrayList<>();

    private StaggeredExecutor() {}

    public void addStaggeredRunnable(@NonNull Runnable callback, double period) {
        toSchedule.add(new StaggeredRunnable(callback, period));
    }

    public void scheduleAll() {
        double avgPeriod = toSchedule.stream().mapToDouble(sr -> sr.periodSeconds).sum() / toSchedule.size();

        for (int i = 0; i < toSchedule.size(); i++) {
            PeriodicExecutor.getInstance().addPeriodicSeconds(
                    toSchedule.get(i).runnable,
                    toSchedule.get(i).periodSeconds,
                    (avgPeriod / toSchedule.size()) * i
            );
        }
    }

    private record StaggeredRunnable(Runnable runnable, double periodSeconds) {}
}
