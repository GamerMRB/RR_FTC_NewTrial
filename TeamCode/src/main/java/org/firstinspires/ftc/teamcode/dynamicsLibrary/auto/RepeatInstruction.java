package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class RepeatInstruction {
    int count;
    Instruction instruction;

    int instructionIndex = 0;
    boolean startInstruction = true;

    public RepeatInstruction(Instruction instruction, int count){
        this.count = count;
        this.instruction = instruction;
    }

    public void start(){
        instructionIndex = 0;
        startInstruction = true;
    }

    public boolean Update(UscOpMode opMode){
        if(startInstruction){
            if(instructionIndex == count){
                return true;
            }
            this.instruction.start(opMode);
            startInstruction = false;
        }
        boolean endInstruction = instruction.update(opMode);
        if(endInstruction){
            instruction.end(opMode);
            instructionIndex++;
            startInstruction = true;
        }
        return false;
    }


}
