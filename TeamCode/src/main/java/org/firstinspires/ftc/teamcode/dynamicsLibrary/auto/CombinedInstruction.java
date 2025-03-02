package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

import java.util.Arrays;

public class CombinedInstruction extends Instruction {
    Instruction[] instructions;
    boolean[] instructionsFinished;
    public CombinedInstruction(Instruction[] instructions){
        this.instructions = instructions;
        instructionsFinished = new boolean[this.instructions.length];
    }

    public void start(UscOpMode opMode) {
        Arrays.fill(instructionsFinished, false);
        for(Instruction instruction : instructions){
            instruction.start(opMode);
        }
    }

    public boolean update(UscOpMode opMode) {
        boolean done = true;
        for(int i = 0; i < instructions.length; i++){
            if(instructionsFinished[i]){
                continue;
            }
            boolean instructionDone = instructions[i].update(opMode);
            if(instructionDone){
                instructions[i].end(opMode);
                instructionsFinished[i] = true;
            }else{
                done = false;
            }
        }
        return done;
    }
}
