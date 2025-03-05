package frc.lib;

import com.sun.management.GarbageCollectionNotificationInfo;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ResourceMonitor {
    private static ResourceMonitor instance;

    public static ResourceMonitor getInstance() {
        if (instance == null) {
            instance = new ResourceMonitor();
        }
        return instance;
    }

    ResourceMonitor() {
        GCMonitor.registerGCListener();

        lastGcTimeNano = System.nanoTime();
        lastGcTime = Timer.getFPGATimestamp();
    }

    //    private final long vmStartTime = ManagementFactory.getRuntimeMXBean().getUptime();;

    private long lastGcTimeNano;
    private double lastGcTime;

    public void update() {
        Logger.recordOutput("gc/timestamp", (System.nanoTime() - lastGcTimeNano) * 1e-6);
        lastGcTimeNano = System.nanoTime();
    }

    public void recordGcEvent(GarbageCollectionNotificationInfo info) {
        Logger.recordOutput("gc/duration", info.getGcInfo().getDuration());
        Logger.recordOutput(
                "gc/memoryFreedMb",
                (info.getGcInfo().getMemoryUsageBeforeGc().size()
                        - info.getGcInfo().getMemoryUsageAfterGc().size() * 1e-6));
        lastGcTimeNano = System.nanoTime();
        lastGcTime = Timer.getFPGATimestamp();
    }
}
