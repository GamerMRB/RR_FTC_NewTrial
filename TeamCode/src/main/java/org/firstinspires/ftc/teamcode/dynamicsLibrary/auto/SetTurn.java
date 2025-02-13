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
        opMode.vel(0,0,deltaEpsilon * 2000);

        return Math.abs(deltaEpsilon) <= 0.01;
    }
}