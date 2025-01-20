package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

        double armTarget = 0;
        boolean armPowered = false;

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
            if (this.gamepad2.right_trigger > 0){
                armPivot.setPower(0.5 * this.gamepad2.right_trigger);
                armPowered = true;
            }
            else if (this.gamepad2.left_trigger > 0){
                armPivot.setPower(-0.05 * this.gamepad2.left_trigger);
                armPowered = true;
            }
            else if (armPowered){
                armPivot.setPower((armPivot.getVelocity()/Math.abs(armPivot.getVelocity())) * 0.05);
                armPowered = false;
            }
            if (this.gamepad2.x || this.gamepad2.dpad_down){ // Contract
                armSlide.setVelocity(-3500);
            }
            else {
                armSlide.setVelocity(0.0);
            }
            if (this.gamepad2.y || this.gamepad2.dpad_up){ // Expand
                armSlide.setVelocity(3500);
            }
            else {
                armSlide.setVelocity(0.0);
            }
            if (this.gamepad2.a || this.gamepad2.left_bumper){
                leftClaw.setPosition(LEFT_OPEN);
                rightClaw.setPosition(RIGHT_OPEN);
            }
            if (this.gamepad2.b || this.gamepad2.right_bumper){
                leftClaw.setPosition(LEFT_CLOSE);
                rightClaw.setPosition(RIGHT_CLOSE);
            }
            telemetry.addLine("Arm pivot position: " + armPivot.getCurrentPosition());
            telemetry.addLine("Arm slide position: " + armSlide.getCurrentPosition());


            telemetry.update();
        }
    }
}