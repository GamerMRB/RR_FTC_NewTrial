package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class SetTurn extends Instruction {
    /**
     * Negative radians is right, and positive radians is left
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
            opMode.frontLeft.setPower(-1);
            opMode.frontRight.setPower(1);
            opMode.backLeft.setPower(-1);
            opMode.backRight.setPower(1);
            if (deltaEpsilon <= theta / 3) {
                for (int i = 2; i <= 16; i*= 2) {
                    opMode.frontLeft.setPower((double) -1 / i);
                    opMode.frontRight.setPower((double) 1 / i);
                    opMode.backLeft.setPower((double) -1 / i);
                    opMode.backRight.setPower((double) 1 / i);
                }
            }
        } else {
            opMode.frontLeft.setPower(1);
            opMode.frontRight.setPower(-1);
            opMode.backLeft.setPower(1);
            opMode.backRight.setPower(-1);
            if (deltaEpsilon <= theta / 3) {
                for (int i = 2; i <= 16; i*= 2) {
                    opMode.frontLeft.setPower((double) 1 / i);
                    opMode.frontRight.setPower((double) -1 / i);
                    opMode.backLeft.setPower((double) 1 / i);
                    opMode.backRight.setPower((double) -1 / i);
                }
            }
        }

        opMode.telemetry.clear();
        opMode.telemetry.addData("Heading", deltaEpsilon);
        opMode.telemetry.update();

        return Math.abs(deltaEpsilon) == 0;
    }
}
