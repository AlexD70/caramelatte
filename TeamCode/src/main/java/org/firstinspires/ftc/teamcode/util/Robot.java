package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Controller;

public class Robot {
    private static Robot robotInst = null;
    public final MecanumDriveEx drive;
    public final HuskyLensDetection husky;
    public final Arm arm;
    public final Lifter lifter;
    public final Camera camera;
    public final PlaneLauncher launcher;
    public final Intake intake;

    public Robot(@NonNull HardwareMap hardwareMap){
        drive = new MecanumDriveEx(hardwareMap);
        husky = new HuskyLensDetection(hardwareMap, "husky");
        arm = new Arm(hardwareMap);
        lifter = new Lifter(hardwareMap);
        camera = new Camera(hardwareMap, "cam");
        launcher = new PlaneLauncher(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void update(){
        arm.update();
        lifter.update();
        drive.update();
    }

    public void endAutonomous(){}

    public void initAutonomous(){
        // camera.init();
        // camera.startAsync();
        // husky.init();
        // arm.init();
        // lifter.init();
        // enable bulk reads???
    }

    public void initTeleOp(){
        // launcher.init()
        // lifter.goToInitPos();
        // lifter.reset();
        // arm.goToInitPos();
        // arm.reset();
        // drive.odometry ???
    }

    public void setGlobal(){
        robotInst = this;
    }

    public static Robot getRobotInstance(HardwareMap hardwareMap){
        if (robotInst == null) {
            new Robot(hardwareMap).setGlobal();
        }
        return robotInst;
    }
}
