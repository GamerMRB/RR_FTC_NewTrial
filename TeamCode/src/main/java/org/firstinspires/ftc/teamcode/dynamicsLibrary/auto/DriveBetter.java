package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    double then;
    public DriveBetter(Path path){
        this.path = path;
        movementPID = new PID2(12, 0, 0);
    }
    public void start(UscOpMode opMode){
        startAngle = opMode.getYaw();
        lastEncoderReadings = opMode.getEncoderReadings().scale(1 / opMode.TICKS_PER_REVOLUTION * opMode.WHEEL_CIRCUMFERENCE);
        startTime = opMode.getRuntime();
        opMode.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        movementPID.reset();
        then = opMode.getRuntime();
    }
    public boolean update(UscOpMode opMode){
        double now = opMode.getRuntime() - startTime;
        double dt = now - then;
        then = now;

        Vec2 targetPos = path.pos(now);
        Vec2 targetVel = path.vel(now);

        // Update current position
        DrivetrainValues currentEncoderReadings = opMode.getEncoderReadings().scale(1 / opMode.TICKS_PER_REVOLUTION * opMode.WHEEL_CIRCUMFERENCE);
        Position change = currentEncoderReadings.sub(lastEncoderReadings).toPosition();
        lastEncoderReadings = currentEncoderReadings;
        currentPos = currentPos.append(change);
        double currentDirection = opMode.getYaw();
        currentPos = Position.va(currentPos.disp, currentDirection);

        Vec2 diff = targetPos.sub(currentPos.disp);
        movementPID.update(diff, dt);

        opMode.telemetry.addLine(diff.toString());
        opMode.telemetry.addLine(String.valueOf(opMode.getYaw()));
        opMode.telemetry.addLine(diff.toString());
        opMode.telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("vel", opMode.getVelocities().toPosition().disp.mag() / opMode.TICKS_PER_REVOLUTION * opMode.WHEEL_CIRCUMFERENCE);
        packet.put("targetVel", targetVel.mag());
        packet.put("pos", currentPos.disp.mag());
        packet.put("targetPos", targetPos.mag());


        FtcDashboard dash = FtcDashboard.getInstance();
        dash.sendTelemetryPacket(packet);

        opMode.vel(Position.v(targetVel.add(movementPID.getPow()).rotate(-currentDirection)).scale(1 / opMode.WHEEL_CIRCUMFERENCE * opMode.TICKS_PER_REVOLUTION));

        return now > path.totTime;
    }
}
