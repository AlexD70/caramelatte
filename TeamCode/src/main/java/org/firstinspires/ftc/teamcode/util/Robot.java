package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private static Robot robotInst = null;
    public final MecanumDriveEx drive;
    //public final HuskyLensDetection husky;
    public final Arm arm;
    public final Lifter lifter;
    //public final Camera camera;
    public final PlaneLauncher launcher;
    public final Intake intake;
    public Telemetry telemetry;

    public boolean debugMode = false;

    public boolean isHanged = false;
    public enum RobotTeleOpStates {
        RUNNING, HANGING_MODE, WAITING, STANDBY
    }
    public RobotTeleOpStates teleopstate = RobotTeleOpStates.STANDBY;

    public void setState(RobotTeleOpStates state){
        teleopstate = state;
    }
    public RobotTeleOpStates getTeleOpState(){
        return teleopstate;
    }

    public Robot(@NonNull HardwareMap hardwareMap){
        drive = new MecanumDriveEx(hardwareMap);
        //husky = new HuskyLensDetection(hardwareMap, "husky");
        arm = new Arm(hardwareMap);
        lifter = new Lifter(hardwareMap);
        //camera = new Camera(hardwareMap, "cam");
        launcher = new PlaneLauncher(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void update(){
        arm.update(telemetry);
        lifter.update();
        if(debugMode){
            arm.printDebug(telemetry);
            lifter.printDebug(telemetry);
        }
        //drive.update();
    }

    public void setTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void toggleDebug(){
        debugMode = !debugMode;
    }
    public boolean isInDebugMode(){
        return debugMode;
    }

    public void initTeleOp(){
        arm.setArmTarget(Arm.ArmPositions.COLLECT);
        RobotTeleOpActions.initActions();
    }

    public void preserveMechanismPositions(){
        throw new RuntimeException("Unimplemented!");
    }

    public void setGlobal(){
        robotInst = this;
    }

    public static Robot getRobotInstance(HardwareMap hardwareMap){
        if (robotInst == null) {
            new Robot(hardwareMap).setGlobal();
        }

        if(hardwareMap == null){
            throw new RuntimeException("Problem initializing! Try performing RobotSetup first!");
        }

        return robotInst;
    }
}
