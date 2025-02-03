package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SeriesInstruction extends Instruction {
    Instruction[] instructions;
    int instructionIndex;
    boolean startInstruction;
    public SeriesInstruction(Instruction[] instructions){
        this.instructions = instructions;
    }

    public void start(UscOpMode opMode) {
        instructionIndex = 0;
        startInstruction = true;
    }
    public boolean update(UscOpMode opMode){
        if(startInstruction){
            if(instructionIndex == instructions.length){
                return true;
            }
            this.instructions[instructionIndex].start(opMode);
            startInstruction = false;
        }
        boolean endInstruction = instructions[instructionIndex].update(opMode);
        if(endInstruction){
            this.instructions[instructionIndex].end(opMode);
            instructionIndex++;
            startInstruction = true;
        }
        return false;
    }
}
