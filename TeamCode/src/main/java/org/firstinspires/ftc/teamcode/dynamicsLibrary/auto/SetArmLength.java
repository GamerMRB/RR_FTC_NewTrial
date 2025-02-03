package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetArmLength extends Instruction{
    int targetPos;
    public SetArmLength(int length){
        targetPos = length;
    }
    public void start(UscOpMode opMode) {
//        opMode.armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        if (targetPos < 0) {
//            opMode.armSlide.setPower(-1);
//        } else {
            opMode.armSlide.setPower(1);
//        }
        opMode.armSlide.setTargetPosition((int) (targetPos * opMode.TICKS_PER_REVOLUTION/ ((48 * Math.PI * 2) / 25.4)));
        opMode.armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean update(UscOpMode opMode){
        opMode.telemetry.clear();
        opMode.telemetry.addLine(Boolean.toString(opMode.armSlide.isBusy()));
        opMode.telemetry.addLine(String.valueOf(opMode.armSlide.getCurrentPosition()));
        opMode.telemetry.addLine(String.valueOf(opMode.armSlide.getTargetPosition()));
        opMode.telemetry.addLine(opMode.armSlide.getMode().toString());
        opMode.telemetry.addLine(String.valueOf(opMode.armSlide.getPower()));
        opMode.telemetry.update();
        return !opMode.armSlide.isBusy();
    }

    public void end(UscOpMode opMode) {
        opMode.armSlide.setPower(0);
        opMode.armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
