package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;

public class Robot implements Updateable {
    public Arm arm;
    public Intake intake;
    public Outtake outtake;
    public PlaneLauncher launcher;
    public Lifter lift;
    public SampleMecanumDrive3 drive;

    public void init(HardwareMap hwmap){
        arm = new Arm(hwmap);
        intake = new Intake(hwmap);
        outtake = new Outtake(hwmap);
        launcher = new PlaneLauncher(hwmap);
        lift = new Lifter(hwmap);
        drive = new SampleMecanumDrive3(hwmap);
    }

    @Override
    public void update() {
        MiscActions.bulkUpdate(arm, intake, outtake, launcher, lift, drive);
    }

    public void printDebug(Telemetry telemetry){
        MiscActions.bulkPrint(telemetry, arm, intake, outtake, launcher, lift);
    }
}
