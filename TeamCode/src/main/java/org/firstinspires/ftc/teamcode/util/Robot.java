package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.Controller;

public class Robot {
    private static Robot robotInst = null;
    public final MecanumDrive drive;
    public final HuskyLensDetection husky;
    public final Arm arm;
    public final Lifter lifter;
    public final Camera camera;
    public final PlaneLauncher launcher;
    public final Intake intake;
    public Controller ctrl1, ctrl2;
    private LinearOpMode activeOpmode;

    public Robot(@NonNull LinearOpMode opmode){
        drive = new MecanumDriveEx(opmode.hardwareMap);
        husky = new HuskyLensDetection(opmode.hardwareMap, "husky");
        arm = new Arm(opmode.hardwareMap, "arm");
        lifter = new Lifter(opmode.hardwareMap, "left", "right");
        camera = new Camera(opmode.hardwareMap, "cam");
        launcher = new PlaneLauncher(opmode.hardwareMap, "plane");
        intake = new Intake(opmode.hardwareMap, "crsleft", "crsright", "angleAdjust");
        changeOpMode(opmode);
    }

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

    public static Robot getRobotInstance(){
        return robotInst;
    }

    public void changeOpMode(LinearOpMode opmode){
        activeOpmode = opmode;
        ctrl1 = new Controller(opmode.gamepad1);
        ctrl2 = new Controller(opmode.gamepad2);
    }


}
