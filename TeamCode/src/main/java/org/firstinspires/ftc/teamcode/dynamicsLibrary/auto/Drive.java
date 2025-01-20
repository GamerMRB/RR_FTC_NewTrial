package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class Drive extends Instruction {
    Vec2 diff;
    public Drive(Vec2 diff){
        this.diff = diff;
    }

    public boolean update(UscOpMode opMode){
        double angle = diff.angle();
        opMode.frontLeft.setVelocity((opMode.MAX_VELOCITY * ((Math.sin(angle) + Math.cos(angle))/Math.sqrt(2))));
        opMode.frontRight.setVelocity((opMode.MAX_VELOCITY * ((Math.cos(angle) - Math.sin(angle))/Math.sqrt(2))));
        opMode.backLeft.setVelocity((opMode.MAX_VELOCITY * ((Math.cos(angle) - Math.sin(angle))/Math.sqrt(2))));
        opMode.backRight.setVelocity((opMode.MAX_VELOCITY * ((Math.sin(angle) + Math.cos(angle))/Math.sqrt(2))));
        double avgPos = (double) (opMode.frontLeft.getCurrentPosition() + opMode.frontRight.getCurrentPosition() + opMode.backLeft.getCurrentPosition() + opMode.backRight.getCurrentPosition()) /4;
        return (Math.abs(((avgPos / opMode.TICKS_PER_REVOLUTION) * opMode.WHEEL_CIRCUMFERENCE) - (diff.mag() * Math.cos(angle))) <= 10);
    }
}
