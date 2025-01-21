package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class Drive extends Instruction {
    Vec2 diff;
    boolean strafeActive = false;
    public Drive(Vec2 diff){
        this.diff = diff;

    }
    // TODO: Make it so that this only has to do one movement to get to final position
    public void start(UscOpMode opMode){
        int targetPos = (int) diff.x;
        for (DcMotorEx motor: opMode.drivetrain){
            motor.setTargetPosition(motor.getCurrentPosition() + targetPos);
            motor.setPower(0.75);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public boolean update(UscOpMode opMode){
        if (opMode.frontLeft.isBusy() && !strafeActive){
            opMode.telemetry.addData("Drive Operation:", "Driving...");
            opMode.telemetry.update();
            return false;
        }
        else if(!opMode.frontLeft.isBusy() && !strafeActive){
            int targetPos = (int)diff.y;
            opMode.frontLeft.setTargetPosition(opMode.frontLeft.getCurrentPosition() + targetPos);
            opMode.frontRight.setTargetPosition(opMode.frontRight.getCurrentPosition() - targetPos);
            opMode.backLeft.setTargetPosition(opMode.backLeft.getCurrentPosition() - targetPos);
            opMode.backRight.setTargetPosition(opMode.backRight.getCurrentPosition() + targetPos);
            for (DcMotorEx motor: opMode.drivetrain){
                motor.setPower(0.75);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            strafeActive = true;
            return false;
        }
        else if (opMode.frontLeft.isBusy() && strafeActive){
            opMode.telemetry.addData("Drive Operation:", "Strafing...");
            opMode.telemetry.update();
            return false;
        }
        else{
            return true;
        }
    }
    public void end(UscOpMode opMode){
        opMode.telemetry.addData("Drive Operation:", "Finished");
        opMode.pow(0,0,0);
    }
}

