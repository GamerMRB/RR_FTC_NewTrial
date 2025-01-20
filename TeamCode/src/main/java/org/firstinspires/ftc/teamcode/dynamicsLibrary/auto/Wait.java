package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

public class Wait extends Instruction{
    long time;
    long targetTime;
    boolean timeCheck = true;
    public Wait(long nanoTime){
        this.time = nanoTime;
    }

    public boolean update(){
        if (timeCheck){
            targetTime = System.nanoTime() + time;
            timeCheck = false;
        }
        return (System.nanoTime() > targetTime);
    }
}
