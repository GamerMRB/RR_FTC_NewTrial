package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetClaw extends Instruction {
    /**
     * true = opening claw, while false = closing claw
     */
    boolean pos;
    double startTime;
    public SetClaw(boolean pos) {
        this.pos = pos;
    }
    public void start(UscOpMode opMode) {
        startTime = opMode.getRuntime();
        if (pos) {
            opMode.leftClaw.setPosition(opMode.LEFT_CLOSE);
            opMode.rightClaw.setPosition(opMode.RIGHT_CLOSE);
        } else {
            opMode.leftClaw.setPosition(opMode.LEFT_OPEN);
            opMode.rightClaw.setPosition(opMode.RIGHT_OPEN);
        }
    }

    public boolean update(UscOpMode opMode) {
        return (opMode.getRuntime() - startTime) >= 0.75;
    }
}
