package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetArm extends Instruction{
    /**
     * Angle to go
     */
    double theta;
    public SetArm(double theta) {
        this.theta = theta;
    }

    public void start(UscOpMode opMode) {
        opMode.armPivot.setTargetPosition((int) (theta * 538 / (2 * Math.PI)));
        opMode.armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean update(UscOpMode opMode) {
        return opMode.armPivot.isBusy();
    }
}
