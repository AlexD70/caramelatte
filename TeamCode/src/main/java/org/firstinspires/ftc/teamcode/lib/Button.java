package org.firstinspires.ftc.teamcode.lib;

import androidx.core.util.Supplier;

public class Button {
    private final Supplier<Boolean> fn;
    private boolean isDown = false, isReleased;
    private int count = 0;

    public Button(Supplier<Boolean> f){
        fn = f;
    }

    public void update(){
        isDown = fn.get();
        isReleased = !isDown && (count > 0);
        count = (isDown)?(count + 1):(0);
    }

    public boolean isDown(){
        return isDown;
    }

    public boolean isPressed(){
        return count == 1;
    }

    public boolean isReleased(){
        return isReleased;
    }
}
