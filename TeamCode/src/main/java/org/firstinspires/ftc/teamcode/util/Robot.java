package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    private static Robot robotInst = null;
    public MecanumDriveEx drive;
    public SampleMecanumDrive rr;
    public SampleMecanumDrive drive2;
    //public final HuskyLensDetection husky;
    public final Arm arm;
    public final Lifter lifter;
    //public final Camera camera;
    public final PlaneLauncher launcher;
    public final Intake intake;
    public Telemetry telemetry;
    public ColorSensor sensor;

    public boolean debugMode = false;

    public boolean isHanged = false;
    public enum RobotTeleOpStates {
        RUNNING, HANGING_MODE, WAITING, STANDBY
    }
    public RobotTeleOpStates teleopstate = RobotTeleOpStates.STANDBY;

    public enum RobotAutoStates {
        RUNNING, WAITING, STANDBY
    }

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
        RobotTeleOpActions.initActions(this);
    }

    public void initTeleOpWithSensor(HardwareMap hwmap){
        initTeleOp();
        sensor = hwmap.get(ColorSensor.class, "sensor");
        sensor.enableLed(false);
    }

    public void initTeleOpV2(HardwareMap hwmap){
        drive2 = new SampleMecanumDrive(hwmap);
        arm.setArmTarget(Arm.ArmPositions.COLLECT);
        RobotTeleOpActions.initActions(this);
    }

    public void initAuto(HardwareMap hwmap, LinearOpMode opmode, RobotAutoActions.Side side){
        rr = new SampleMecanumDrive(hwmap);
        arm.setArmTarget(Arm.ArmPositions.COLLECT);
        RobotAutoActions.initAutoActions(this, side, opmode);
    }

    public void preserveMechanismPositions(){
        throw new RuntimeException("Unimplemented!");
    }

    public void setGlobal(){
        robotInst = this;
    }

    public static Robot getRobotInstance(HardwareMap hardwareMap){
        if(robotInst == null && hardwareMap == null){
            throw new RuntimeException("Problem initializing");
        }

        if (robotInst == null) {
            new Robot(hardwareMap).setGlobal();
        }

        return robotInst;
    }
}
