package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

/**
 * Negative radians is right, and positive radians is left
 */
public class SetTurn extends Instruction {
    double theta;
    double deltaEpsilon;
    public SetTurn(double theta) {
        this.theta = theta;
    }

    public void start(UscOpMode opMode) {
        opMode.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean update(UscOpMode opMode) {
        deltaEpsilon = opMode.simplifyAngle(theta - opMode.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double restoring = 4000;
        double max = 2000;

        opMode.turnVel = Math.max(-max, Math.min(max, deltaEpsilon * restoring));
        opMode.updateVel();

        return Math.abs(deltaEpsilon) <= 0.01;
    }

    public void end(UscOpMode opMode){
        opMode.turnVel = 0;
        opMode.updateVel();
    }
}