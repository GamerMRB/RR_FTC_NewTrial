package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetTurn extends Instruction {
    /**
     * Negative radians is right, and positive radians is left (If theta is negative, PLEASE PLEASE PLEASE make direction negative)
     */
    double theta;
    double imuHeading;
    int direction;
    public SetTurn(double theta, int direction) {
        this.theta = theta;
        this.direction = direction;
    }

    public void start(UscOpMode opMode) {
        opMode.frontLeft.setPower(-direction);
        opMode.frontRight.setPower(direction);
        opMode.backLeft.setPower(-direction);
        opMode.backRight.setPower(direction);
    }

    public boolean update(UscOpMode opMode) {
        imuHeading = (opMode.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)).firstAngle;
        return Math.abs(imuHeading - theta) <= 0.1;
    }
}
