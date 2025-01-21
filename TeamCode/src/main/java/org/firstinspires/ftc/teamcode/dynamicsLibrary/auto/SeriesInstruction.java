package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SeriesInstruction extends Instruction {
    Instruction[] instructions;
    int instructionIndex = 0;
    boolean startInstruction = true;
    public SeriesInstruction(Instruction[] instructions){
        this.instructions = instructions;
    }
    public boolean update(UscOpMode opMode){
        if(startInstruction){
            if(instructionIndex == this.instructions.length){
                return true;
            }
            this.instructions[instructionIndex].start(opMode);
            startInstruction = false;
        }
        boolean endInstruction = this.instructions[instructionIndex].update(opMode);
        if(endInstruction){
            this.instructions[instructionIndex].end(opMode);
            instructionIndex++;
            startInstruction = true;
        }
        return false;
    }
}
