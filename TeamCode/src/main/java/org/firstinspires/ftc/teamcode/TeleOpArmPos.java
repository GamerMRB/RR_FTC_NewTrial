package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpArmPos extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();
        double speedX = 0.75 * SPEED_MAX;
        double currentX;
        double currentY;

        double armTarget = 0;
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

            // Claw
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
