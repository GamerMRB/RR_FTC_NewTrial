package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.DrivetrainValues;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Position;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

public class DriveBetter extends Instruction {
    Position target;
    double startAngle;
    Position currentPos = Position.zero;
    DrivetrainValues lastEncoderReadings;
    public DriveBetter(Position target){
        this.target = target;
    }
    public void start(UscOpMode opMode){
        startAngle = opMode.getYaw();
        lastEncoderReadings = opMode.getEncoderReadings();
    }
    public boolean update(UscOpMode opMode){

        DrivetrainValues currentEncoderReadings = opMode.getEncoderReadings();
        Position change = currentEncoderReadings.sub(lastEncoderReadings).toPosition();
        currentPos = currentPos.append(change);

        double currentDirection = opMode.getYaw();
        currentPos = Position.va(currentPos.disp, currentDirection);

//        Position diff = target.


        return false;
    }
}
