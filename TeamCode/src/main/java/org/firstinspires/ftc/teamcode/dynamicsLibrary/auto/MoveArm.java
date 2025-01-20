package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class MoveArm extends CombinedInstruction {
    MoveArm(Vec2 pos){
        super(new Instruction[]{
            new SetArmLength(pos.x),
            new SetArmAngle(pos.y),
        });
    }
}
