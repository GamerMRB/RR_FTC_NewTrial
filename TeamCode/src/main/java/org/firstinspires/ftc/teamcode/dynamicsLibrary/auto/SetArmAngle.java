package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetArmAngle extends Instruction{
    /**
     * Angle to go
     */
    double theta;
    public SetArmAngle(double theta) {
        this.theta = theta;
    }

    public void start(UscOpMode opMode) {
        opMode.armPivot.setPower(0.5);
        opMode.armPivot.setTargetPosition((int) (theta * 538 / (2 * Math.PI)));
        opMode.armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean update(UscOpMode opMode) {
        return !opMode.armPivot.isBusy();
    }
}
