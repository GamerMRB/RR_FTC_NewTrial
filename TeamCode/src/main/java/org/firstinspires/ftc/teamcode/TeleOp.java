package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();
        double speedX = 0.75 * SPEED_MAX;
        double currentX;
        double currentY;
        double armTarget = 0;
        double armZero = armSlide.getCurrentPosition();
        boolean armPowered = false;

        while (opModeIsActive()) {
            // Drive
            telemetry.addData("Turn: ", this.gamepad1.right_stick_x);
            telemetry.addData("Throttle: ", this.gamepad1.left_stick_y);
            pow(
                    -speedX*scaleMovement(gamepad1.left_stick_y),
                    -speedX*((gamepad1.right_bumper ? 1 : 0) - (gamepad1.left_bumper ? 1 : 0)),
                    -speedX*scaleMovement(gamepad1.right_stick_x)
            );
            // Arm
//            if (this.gamepad2.right_trigger > 0){
//                armPivot.setPower(0.5 * this.gamepad2.right_trigger);
//                armPowered = true;
//
//            }
//            else if (this.gamepad2.left_trigger > 0){
//                armPivot.setPower(-0.10 * this.gamepad2.left_trigger);
//                armPowered = true;
//            }
//            else if (armPowered){
//                armPivot.setPower(
//                         0.02 * ARM_MASS * GRAVITY * ((ARM_RANGE_METERS/ARM_RANGE_TICKS) * (armSlide.getCurrentPosition()-armZero) + ARM_MIN_METERS) * Math.cos((Math.PI/TICKS_PER_PI) * (armPivot.getCurrentPosition() - ARM_PIVOT_ZERO))
//                );
//                armPowered = false;
//            }

            double addedPow = 0.4*(this.gamepad2.right_trigger - this.gamepad2.left_trigger);
            armPivot.setPower(
                    addedPow + 0.02 * ARM_MASS * GRAVITY * ((ARM_RANGE_METERS/ARM_RANGE_TICKS) * (armSlide.getCurrentPosition()-armZero) + ARM_MIN_METERS) * Math.cos((Math.PI/TICKS_PER_PI) * (armPivot.getCurrentPosition() - ARM_PIVOT_ZERO))
            );

            telemetry.addData("Arm Pos: ", armPivot.getCurrentPosition());
            telemetry.addData("Arm Pivot Power: ", armPivot.getPower());
            if (this.gamepad2.x || this.gamepad2.dpad_down){ // Contract
                armSlide.setVelocity(-3500);
            }
            else {
                armSlide.setVelocity(0.0);
            }
            if ((this.gamepad2.y || this.gamepad2.dpad_up) && ((armSlide.getCurrentPosition() - armZero) *  Math.cos((Math.PI/TICKS_PER_PI) * (armPivot.getCurrentPosition() - ARM_PIVOT_ZERO)) < 2180)){ // Expand
                armSlide.setVelocity(3500);
            }
            else {
                armSlide.setVelocity(0.0);
            }
            if (this.gamepad2.a || this.gamepad2.right_bumper){
                leftClaw.setPosition(LEFT_OPEN);
                rightClaw.setPosition(RIGHT_OPEN);
            }
            if (this.gamepad2.b || this.gamepad2.left_bumper){
                leftClaw.setPosition(LEFT_CLOSE);
                rightClaw.setPosition(RIGHT_CLOSE);
            }
            telemetry.addLine("Arm pivot position: " + armPivot.getCurrentPosition());
            telemetry.addLine("Arm slide position: " + armSlide.getCurrentPosition());


            telemetry.update();
        }
    }
}