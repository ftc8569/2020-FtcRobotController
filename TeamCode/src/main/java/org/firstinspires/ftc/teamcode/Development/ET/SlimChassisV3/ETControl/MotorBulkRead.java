package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class MotorBulkRead {
    protected static LynxModule.BulkCachingMode mode;
    protected static List<LynxModule> allHubs;

    public static void MotorBulkMode(LynxModule.BulkCachingMode bcm, HardwareMap hw) {
        mode = bcm;
        allHubs = hw.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(bcm);
        }
    }

    public static void clearCache() {
        if(mode == LynxModule.BulkCachingMode.MANUAL) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
    }

}
