package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetTurn extends Instruction {
    /**
     * Negative radians is right, and positive radians is left (If theta is negative, PLEASE PLEASE PLEASE make direction negative(same with positive))
     */
    double theta;
    double imuHeading;
    public SetTurn(double theta) {
        this.theta = theta;
    }

    public void start(UscOpMode opMode) {
        if (theta > 0) {
            opMode.frontLeft.setPower(-1);
            opMode.frontRight.setPower(1);
            opMode.backLeft.setPower(-1);
            opMode.backRight.setPower(1);
        } else {
            opMode.frontLeft.setPower(1);
            opMode.frontRight.setPower(-1);
            opMode.backLeft.setPower(1);
            opMode.backRight.setPower(-1);
        }

    }

    public boolean update(UscOpMode opMode) {
        imuHeading = (opMode.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)).firstAngle;
        return Math.abs(imuHeading - theta) <= 0.1;
    }
}
