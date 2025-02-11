package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

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
        opMode.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean update(UscOpMode opMode) {
        deltaEpsilon = opMode.simplifyAngle(theta - opMode.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        if (deltaEpsilon > 0) {
            opMode.frontLeft.setPower(-0.5 * deltaEpsilon);
            opMode.frontRight.setPower(0.5 * deltaEpsilon);
            opMode.backLeft.setPower(-0.5 * deltaEpsilon);
            opMode.backRight.setPower(0.5 * deltaEpsilon);
        } else {
            opMode.frontLeft.setPower(0.5 * deltaEpsilon);
            opMode.frontRight.setPower(-0.5 * deltaEpsilon);
            opMode.backLeft.setPower(0.5 * deltaEpsilon);
            opMode.backRight.setPower(-0.5 * deltaEpsilon);
        }

        opMode.telemetry.clear();
        opMode.telemetry.addData("Heading", deltaEpsilon);
        opMode.telemetry.update();

        return Math.abs(deltaEpsilon) <= 0.01;
    }
}
