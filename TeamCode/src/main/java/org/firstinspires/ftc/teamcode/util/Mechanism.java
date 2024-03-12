package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Mechanism extends Updateable {
    int getPosition();
    boolean isBusy();
    void setTarget(int target);
    void printDebug(Telemetry telemetry);
}
