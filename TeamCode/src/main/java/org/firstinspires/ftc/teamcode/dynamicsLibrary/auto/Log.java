package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class Log extends Instruction {
    String message;
    public Log(String message){
        this.message = message;
    }

    public void start(UscOpMode opMode){
        opMode.telemetry.addLine(message);
        opMode.telemetry.update();
    }
}
