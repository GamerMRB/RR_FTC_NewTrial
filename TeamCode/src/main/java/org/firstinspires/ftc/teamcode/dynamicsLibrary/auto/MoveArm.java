package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class MoveArm extends CombinedInstruction {
    public MoveArm(Vec2 pos){
        super(new Instruction[]{
            new SetArmLength((int) pos.mag()),
            new SetArmAngle(pos.angle()),
        });
    }
}
