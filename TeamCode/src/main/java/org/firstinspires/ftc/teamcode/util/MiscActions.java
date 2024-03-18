package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;

public class MiscActions {
    public static void bulkSetThreshes(@NonNull Controller c1, @NonNull Controller c2, double thresh){
        c1.setLeftTriggerThreshold(thresh);
        c1.setRightTriggerThreshold(thresh);
        c2.setRightTriggerThreshold(thresh);
        c2.setLeftTriggerThreshold(thresh);
    }

    public static void bulkUpdate(@NonNull Updateable... updateables){
        for(Updateable u : updateables){
            u.update();
        }
    }

    public static void bulkUpdate(@NonNull Object... objs){
        for(Object o : objs){
            if(o instanceof Updateable){
                ((Updateable) o).update();
            } else if (o instanceof Telemetry){
                ((Telemetry) o).update();
            } else if (o instanceof SampleMecanumDrive3){
                ((SampleMecanumDrive3) o).update();
            }
        }
    }

    public static void bulkPrint(Telemetry telemetry, @NonNull Mechanism... mechsanisms){
        for(Mechanism m : mechsanisms){
            m.printDebug(telemetry);
        }
    }
}
