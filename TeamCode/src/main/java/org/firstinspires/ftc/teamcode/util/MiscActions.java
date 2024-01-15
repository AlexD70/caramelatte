package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class MiscActions {
    public static void bulkSetThreshes(Controller c1, Controller c2, double thresh){
        c1.setLeftTriggerThreshold(thresh);
        c1.setRightTriggerThreshold(thresh);
        c2.setRightTriggerThreshold(thresh);
        c2.setLeftTriggerThreshold(thresh);
    }

    public static void bulkUpdate(Updateable... updateables){
        for(Updateable u : updateables){
            u.update();
        }
    }

    public static void bulkUpdate(Object... objs){
        for(Object o : objs){
            if(o instanceof Updateable){
                ((Updateable) o).update();
            } else if (o instanceof Telemetry){
                ((Telemetry) o).update();
            } else if (o instanceof SampleMecanumDrive){
                ((SampleMecanumDrive) o).update();
            }
        }
    }
}
