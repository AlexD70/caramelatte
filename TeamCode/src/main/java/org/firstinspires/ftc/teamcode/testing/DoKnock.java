package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;

@TeleOp(name = "knock", group = "test")
public class DoKnock extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HuskyLensDetection husky = new HuskyLensDetection(hardwareMap, "husky");

        waitForStart();

        if(husky.lens.knock()){
            telemetry.addLine("OK");
        } else {
            telemetry.addLine("NOK");
        }
        telemetry.update();

        sleep(2000);
        husky.lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while(opModeIsActive()){
            sleep(50);
            HuskyLens.Block[] blocks = husky.lens.blocks();
            int blocklen = blocks.length;
            telemetry.addData("LENGTH ", blocklen);
            if(blocklen == 1){
                telemetry.addData("x", blocks[0].x);
                telemetry.addData("y", blocks[0].y);
                telemetry.addData("w", blocks[0].width);
                telemetry.addData("h", blocks[0].height);
//                if(blocks[0].x + blocks[0].width < 160){ RED FAR
//                    telemetry.addLine("LEFT"); area ~800
//                } else if (blocks[0].x > 120 && blocks[0].width < 50){
//                    telemetry.addLine("CENTER"); area 250-550
//                }


//                if(blocks[0].x > 140){ RED CLOSE
//                    telemetry.addLine("RIGHT");
//                } else if (blocks[0].x + blocks[0].width < 200){
//                    telemetry.addLine("CENTER");
//                }
//                telemetry.addData("area", blocks[0].width * blocks[0].height / 2);
            } else {
//                telemetry.addLine("RIGHT"); RED FAR
//                telemetry.addLine("LEFT"); RED CLOSE
            }
            telemetry.update();
        }

        sleep(10000);
    }
}
