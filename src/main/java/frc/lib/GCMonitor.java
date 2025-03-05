package frc.lib;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import javax.management.*;
import javax.management.openmbean.CompositeData;

import com.sun.management.GarbageCollectionNotificationInfo;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class GCMonitor {
    public static void registerGCListener() {
        List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        for (GarbageCollectorMXBean gcBean : gcBeans) {
            try {
                ObjectName gcName = new ObjectName("java.lang:type=GarbageCollector,name=" + gcBean.getName());
                ManagementFactory.getPlatformMBeanServer().addNotificationListener(gcName, new NotificationListener() {
                    @Override
                    public void handleNotification(Notification notification, Object handback) {
                        // Only process GC notifications.
                        if (notification.getType().equals(GarbageCollectionNotificationInfo.GARBAGE_COLLECTION_NOTIFICATION)) {
                            CompositeData cd = (CompositeData) notification.getUserData();
                            GarbageCollectionNotificationInfo info = GarbageCollectionNotificationInfo.from(cd);

                            ResourceMonitor.getInstance().recordGcEvent(info);
                        }
                    }
                }, null, null);
            } catch (InstanceNotFoundException | MalformedObjectNameException e) {
                e.printStackTrace();
            }
        }
    }
}