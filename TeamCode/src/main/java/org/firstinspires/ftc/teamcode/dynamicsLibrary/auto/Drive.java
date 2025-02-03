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
        int ticks = (int) (((diff.x / (opMode.WHEEL_DIAMETER / 2)) / (2 * Math.PI)) * opMode.TICKS_PER_REVOLUTION);
        for (DcMotorEx motor: opMode.drivetrain){
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(0.75);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public boolean update(UscOpMode opMode){
        if (opMode.frontLeft.isBusy() && !strafeActive){
            return false;
        }
        else if(!opMode.frontLeft.isBusy() && !strafeActive){
            int ticks = (int) (((diff.y / (opMode.WHEEL_DIAMETER / 2)) / (2 * Math.PI)) * opMode.TICKS_PER_REVOLUTION);
            opMode.frontLeft.setTargetPosition(opMode.frontLeft.getCurrentPosition() + ticks);
            opMode.frontRight.setTargetPosition(opMode.frontRight.getCurrentPosition() - ticks);
            opMode.backLeft.setTargetPosition(opMode.backLeft.getCurrentPosition() - ticks);
            opMode.backRight.setTargetPosition(opMode.backRight.getCurrentPosition() + ticks);
            for (DcMotorEx motor: opMode.drivetrain){
                motor.setPower(0.75);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            strafeActive = true;
            return false;
        }
        else return !opMode.frontLeft.isBusy() || !strafeActive;
    }
    public void end(UscOpMode opMode){
        opMode.pow(0,0,0);
    }
}

