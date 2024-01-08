package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.lib.Controller;

public class MiscActions {
    public static void bulkSetThreshes(Controller c1, Controller c2, double thresh){
        c1.setLeftTriggerThreshold(thresh);
        c1.setRightTriggerThreshold(thresh);
        c2.setRightTriggerThreshold(thresh);
        c2.setLeftTriggerThreshold(thresh);
    }
}
