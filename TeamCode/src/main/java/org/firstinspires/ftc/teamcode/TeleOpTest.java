package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

import java.util.ArrayList;

@TeleOp
public class TeleOpTest extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();
        double speedX = 0.75 * SPEED_MAX;
        double strafeSpeedX = STRAFE_SPEED;
        double currentX;
        double currentY;

        while (opModeIsActive()) {
           // Drive
            telemetry.addData("Turn: ", -this.gamepad1.right_stick_x);
            telemetry.addData("Throttle: ", -this.gamepad1.left_stick_y);
            currentX = this.gamepad1.right_stick_x;
            currentY = this.gamepad1.left_stick_y;
            double throttle = -currentY * speedX;
            // Allow second stick to turn also
            double turn = currentX * speedX / 2.0;
            double leftSpeed = -1 * (throttle + turn);
            double rightSpeed = throttle - turn;
            frontLeft.setPower(leftSpeed);
            backLeft.setPower(leftSpeed);
            frontRight.setPower(rightSpeed);
            backRight.setPower(rightSpeed);
            // Strafe
            if (this.gamepad1.left_bumper) {
                frontLeft.setPower(strafeSpeedX);
                frontRight.setPower(strafeSpeedX);
                backLeft.setPower(-strafeSpeedX);
                backRight.setPower(-strafeSpeedX);
            }
            if (this.gamepad1.right_bumper) {
                frontLeft.setPower(-strafeSpeedX);
                frontRight.setPower(-strafeSpeedX);
                backLeft.setPower(strafeSpeedX);
                backRight.setPower(strafeSpeedX);
            }
            // Arm
            if (this.gamepad1.left_trigger > 0){
                armPivot.setPower(1.0);
            }
            else{
                armPivot.setPower(0.0);
            }
            if (this.gamepad1.right_trigger > 0){
                armPivot.setPower(-1.0);
            }
            else{
                armPivot.setPower(0.0);
            }
            if (this.gamepad1.y){
                armSlide.setPower(-1.0);
            }
            else {
                armSlide.setPower(0.0);
            }
            if (this.gamepad1.x){
                armSlide.setPower(1.0);
            }
            else {
                armSlide.setPower(0.0);
            }
            ArrayList<Quaternion> detections = updatePos();
            telemetry.addLine(robotPos.toString());
            for(Quaternion detection : detections){
                telemetry.addLine(detection.toString());
            }
            telemetry.update();
        }
    }
}