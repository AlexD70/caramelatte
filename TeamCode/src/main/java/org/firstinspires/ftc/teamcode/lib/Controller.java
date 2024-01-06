package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

// V2

public class Controller {
    private final Gamepad g;

    public Button square, circle, cross, triangle, share, options;
    public Button dpadUp, dpadDown, dpadLeft, dpadRight;
    public Button bumperRight, bumperLeft;
    public Button leftTriggerButton, rightTriggerButton;

    public double rightStickX, rightStickY; // right
    public double leftStickX, leftStickY; // left

    public double rightTrigger, leftTrigger;
    private double rtrigthresh = 0, ltrigthresh = 0;
    private int rtrigcounter = 0, ltrigcounter = 0;

    public Controller(Gamepad g) {
        this.g = g;
        square = new Button(() -> g.square);
        circle = new Button(() -> g.circle);
        triangle = new Button(() -> g.triangle);
        cross = new Button(() -> g.cross);
        share = new Button(() -> g.share);
        options = new Button(() -> g.options);
        dpadUp = new Button(() -> g.dpad_up);
        dpadDown = new Button(() -> g.dpad_down);
        dpadLeft = new Button(() -> g.dpad_left);
        dpadRight = new Button(() -> g.dpad_right);
        bumperRight = new Button(() -> g.right_bumper);
        bumperLeft = new Button(() -> g.left_bumper);
        leftTriggerButton = new Button(this::isLeftTriggerDown);
        rightTriggerButton = new Button(this::isRightTriggerDown);
    }

    public void update(){
        square.update();
        circle.update();
        triangle.update();
        share.update();
        options.update();
        cross.update();
        dpadUp.update();
        dpadDown.update();
        dpadLeft.update();
        dpadRight.update();
        bumperLeft.update();
        bumperRight.update();

        leftStickX = g.left_stick_x;
        leftStickY = g.left_stick_y;
        rightStickX = g.right_stick_x;
        rightStickY = g.right_stick_y;

        rightTrigger = g.right_trigger;
        leftTrigger = g.left_trigger;

        rtrigcounter = (rightTrigger > rtrigthresh)?(rtrigcounter + 1):(0);
        ltrigcounter = (leftTrigger > ltrigthresh)?(ltrigcounter + 1):(0);
        leftTriggerButton.update();
        rightTriggerButton.update();
    }

    public boolean isButtonDown(Button b){
        return b.isDown();
    }

    public boolean isButtonReleased(Button b){
        return b.isReleased();
   }

    // old once
    public boolean isButtonPressed(Button b){
        return b.isPressed();
   }

    public boolean isLeftTriggerDown(){
        return ltrigcounter > 0;
   }

    public boolean isRightTriggerDown(){
        return rtrigcounter > 0;
   }

    public boolean isRightTriggerPressed(){
        return rtrigcounter == 1;
   }

    public boolean isLeftTriggerPressed(){
        return ltrigcounter == 1;
   }

    public void setLeftTriggerThreshold(double t){
        ltrigthresh = t;
   }

    public void setRightTriggerThreshold(double t){
        rtrigthresh = t;
   }
}
