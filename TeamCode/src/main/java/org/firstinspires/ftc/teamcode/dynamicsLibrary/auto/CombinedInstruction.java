package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class CombinedInstruction extends Instruction {
    Instruction[] instructions;
    CombinedInstruction(Instruction[] instructions){
        this.instructions = instructions;
    }

    public void start(UscOpMode opMode) {
        for(Instruction instruction : instructions){
            instruction.start(opMode);
        }
    }

    public boolean update(UscOpMode opMode) {
        boolean done = true;
        for(Instruction instruction : instructions){
            if(!instruction.update(opMode)){
                done = false;
            }
        }
        return done;
    }
}
