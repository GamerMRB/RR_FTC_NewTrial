package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetArmLength extends Instruction{
    int targetPos;
    ElapsedTime e;
    long startTime = System.nanoTime();

    public SetArmLength(int length){
        targetPos = length;
    }
    public void start(UscOpMode opMode) {
        opMode.armSlide.setPower(1);
        opMode.armSlide.setTargetPosition((int) (targetPos * opMode.TICKS_PER_REVOLUTION/ ((48 * Math.PI * 2) / 25.4)));
        opMode.armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean update(UscOpMode opMode){
        long endTime = System.nanoTime();
        long elapsedTime = endTime - startTime;
        double elapsedTimeInMs = elapsedTime / 1_000_000.0;
        return !opMode.armSlide.isBusy() | elapsedTimeInMs > 1000;
    }
}
