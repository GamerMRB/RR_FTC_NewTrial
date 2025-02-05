package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetArmLength extends Instruction{
    int targetPos;
    public SetArmLength(int length){
        targetPos = length;
    }
    public void start(UscOpMode opMode) {
        opMode.armSlide.setPower(1);
        opMode.armSlide.setTargetPosition((int) (targetPos * opMode.TICKS_PER_REVOLUTION/ ((48 * Math.PI * 2) / 25.4)));
        opMode.armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean update(UscOpMode opMode){
        return !opMode.armSlide.isBusy();
    }
}
