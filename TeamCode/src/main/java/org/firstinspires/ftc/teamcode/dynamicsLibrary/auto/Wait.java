package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

public class Wait extends Instruction{
    long time;
    long targetTime;
    public Wait(long nanoTime){
        this.time = nanoTime;
    }

    public void start(){
        targetTime = System.nanoTime() + time;
    }
    public boolean update(){
        return System.nanoTime() > targetTime;
    }
}
