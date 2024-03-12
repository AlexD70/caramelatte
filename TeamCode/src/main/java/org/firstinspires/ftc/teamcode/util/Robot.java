package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Robot implements Updateable{
    public Arm arm;
    public Intake intake;

    public void init(HardwareMap hwmap){
        //arm = new Arm(hwmap);
        intake = new Intake(hwmap);
    }

    @Override
    public void update() {
        //arm.update();
        intake.update();
    }
}
