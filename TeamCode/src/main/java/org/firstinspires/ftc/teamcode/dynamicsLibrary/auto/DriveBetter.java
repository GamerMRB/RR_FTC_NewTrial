package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.DrivetrainValues;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.PID2;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Position;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;

public class DriveBetter extends Instruction {
    Path path;
    double startAngle;
    Position currentPos = Position.zero;
    DrivetrainValues lastEncoderReadings;
    double startTime;
    PID2 movementPID;
    public DriveBetter(Path path){
        this.path = path;
        movementPID = new PID2(0.1, 0, 0);
    }
    public void start(UscOpMode opMode){
        startAngle = opMode.getYaw();
        lastEncoderReadings = opMode.getEncoderReadings();
        startTime = opMode.getRuntime();
        opMode.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean update(UscOpMode opMode){
        double t = opMode.getRuntime() - startTime;
        Vec2 targetPos = path.pos(t);
        Vec2 targetVel = path.vel(t);

        // Update current position
        DrivetrainValues currentEncoderReadings = opMode.getEncoderReadings();
        Position change = currentEncoderReadings.sub(lastEncoderReadings).toPosition();
        currentPos = currentPos.append(change);
        double currentDirection = opMode.getYaw();
        currentPos = Position.va(currentPos.disp, currentDirection);

        Vec2 diff = targetPos.sub(currentPos.disp);

        opMode.telemetry.addLine(currentPos.toString());
        opMode.telemetry.addLine(targetPos.toString());
        opMode.telemetry.update();
        opMode.vel(Position.v(targetVel));

        return false;
    }
}
