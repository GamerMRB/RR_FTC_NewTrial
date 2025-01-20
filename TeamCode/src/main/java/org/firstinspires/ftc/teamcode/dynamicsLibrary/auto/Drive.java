package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class Drive extends Instruction {
    Vec2 diff;
    public Drive(Vec2 diff){
        this.diff = diff;
    }
    public boolean update(UscOpMode opMode){

    }
}
