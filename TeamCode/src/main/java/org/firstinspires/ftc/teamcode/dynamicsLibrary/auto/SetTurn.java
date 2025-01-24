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
    double deltaEpsilon;
    public SetTurn(double theta) {
        this.theta = theta;
    }

    public void start(UscOpMode opMode) {
        opMode.imu.resetYaw();
    }

    public boolean update(UscOpMode opMode) {
        deltaEpsilon = opMode.simplifyAngle(theta - opMode.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        if (deltaEpsilon > 0) {
            opMode.frontLeft.setPower(-0.5);
            opMode.frontRight.setPower(0.5);
            opMode.backLeft.setPower(-0.5);
            opMode.backRight.setPower(0.5);
            if (deltaEpsilon <= deltaEpsilon / 3) {
                for (int i = 2; i <= 16; i*= 2) {
                    opMode.frontLeft.setPower(-0.5 / i);
                    opMode.frontRight.setPower(0.5 / i);
                    opMode.backLeft.setPower(-0.5 / i);
                    opMode.backRight.setPower(0.5 / i);
                }
            }
        } else {
            opMode.frontLeft.setPower(0.5);
            opMode.frontRight.setPower(-0.5);
            opMode.backLeft.setPower(0.5);
            opMode.backRight.setPower(-0.5);
            if (deltaEpsilon <= deltaEpsilon / 3) {
                for (int i = 2; i <= 16; i*= 2) {
                    opMode.frontLeft.setPower(0.5 / i);
                    opMode.frontRight.setPower(-0.5 / i);
                    opMode.backLeft.setPower(0.5 / i);
                    opMode.backRight.setPower(-0.5 / i);
                }
            }
        }

        opMode.telemetry.clear();
        opMode.telemetry.addData("Heading", deltaEpsilon);
        opMode.telemetry.update();

        return Math.abs(deltaEpsilon) <= 0.25 ;
    }
}
