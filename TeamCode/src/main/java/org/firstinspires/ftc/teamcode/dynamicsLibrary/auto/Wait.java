package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class Wait extends Instruction{
    long time;
    long targetTime;
    public Wait(long nanoTime){
        this.time = nanoTime;
    }

    public void start(UscOpMode opMode){
        targetTime = System.nanoTime() + time;
    }
    public boolean update(UscOpMode opMode){

        return System.nanoTime() > targetTime;
    }
}
