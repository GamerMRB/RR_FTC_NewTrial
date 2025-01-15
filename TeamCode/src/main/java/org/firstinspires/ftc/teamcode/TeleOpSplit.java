package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class TeleOpSplit extends UscOpMode {

    enum ArmPosition {
        LOW_CHAMBER,
        HIGH_CHAMBER,
        LOW_BASKET,
        HIGH_BASKET
    }

    public void runOpMode() {
        Map<ArmPosition, Integer> armRotation = new HashMap<>();
        armRotation.put(ArmPosition.LOW_CHAMBER, 670);
        armRotation.put(ArmPosition.HIGH_CHAMBER, 1200);
        armRotation.put(ArmPosition.LOW_BASKET, 1070);
        armRotation.put(ArmPosition.HIGH_BASKET, 1400);
        int defaultArmRotation = 0;

        Map<ArmPosition, Integer> armLength = new HashMap<>();
        armLength.put(ArmPosition.LOW_CHAMBER, 40);
        armLength.put(ArmPosition.HIGH_CHAMBER, 640);
        armLength.put(ArmPosition.LOW_BASKET, 1825);
        armLength.put(ArmPosition.HIGH_BASKET, 2990);
        int defaultArmLength = 300;

        Map<ArmPosition, String> positionNames = new HashMap<>();
        positionNames.put(ArmPosition.LOW_CHAMBER, "low chamber");
        positionNames.put(ArmPosition.HIGH_CHAMBER, "high chamber");
        positionNames.put(ArmPosition.LOW_BASKET, "low basket");
        positionNames.put(ArmPosition.HIGH_BASKET, "high basket");

        ArmPosition targetPos = ArmPosition.HIGH_CHAMBER;
        boolean armUp = false;
        boolean clawClosed = false;

        boolean leftPressed = false;
        boolean rightPressed = false;
        boolean toggledArm = false;
        boolean toggledClaw = false;

        setUpHardware();
        waitForStart();

        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Starting at",  "%7d", armPivot.getCurrentPosition());
        telemetry.update();

        armPivot.setTargetPosition(defaultArmRotation);
        armSlide.setTargetPosition(defaultArmLength);
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armPivot.setVelocityPIDFCoefficients(2.5, 0.1, 0.2, 0);
//        armPivot.setPositionPIDFCoefficients(1);
        armPivot.setPower(1);
        armSlide.setPower(1);

        while (opModeIsActive()){
            if(gamepad2.dpad_left){
                if(!leftPressed) {
                    targetPos = ArmPosition.values()[(targetPos.ordinal() + 1) % ArmPosition.values().length];
                    leftPressed = true;
                }
            }else{
                leftPressed = false;
            }
            if(gamepad2.dpad_right){
                if(!rightPressed) {
                    targetPos = ArmPosition.values()[(targetPos.ordinal() - 1 + ArmPosition.values().length) % ArmPosition.values().length];
                    rightPressed = true;
                }
            }else{
                rightPressed = false;
            }
            if(gamepad1.a){
                if(!toggledArm){
                    armUp = !armUp;
                    toggledArm = true;
                }
            }else{
                toggledArm = false;
            }
            if(gamepad1.b){
                if(!toggledClaw){
                    clawClosed = !clawClosed;
                    toggledClaw = true;
                    setClaw(clawClosed);
                }
            }else{
                toggledClaw = false;
            }
            if(gamepad1.x){
                armPivot.setMotorDisable();
                armSlide.setMotorDisable();
            }else{
                armPivot.setMotorEnable();
                armSlide.setMotorEnable();
            }
            pow(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            armPivot.setTargetPosition(armUp ? armRotation.get(targetPos) : defaultArmRotation);
            armSlide.setTargetPosition(armUp ? armLength.get(targetPos) : defaultArmLength);

            telemetry.clear();
            telemetry.addLine("Current position: " + armPivot.getCurrentPosition());
            telemetry.addLine("Current distance: " + armSlide.getCurrentPosition());
            telemetry.addLine("Current target: " + (armUp ? armRotation.get(targetPos) : defaultArmRotation));
            telemetry.addLine("Target name: " + positionNames.get(targetPos));

            telemetry.update();

            sleep(100);
        }
    }
}
