package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.DriveEasier;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Wait;


@Autonomous
public class DriveBetterTest extends UscOpMode {

    public void runOpMode() {
        setUpHardware();

        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetIMU();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("vel", 0);
        packet.put("targetVel", 0);
        packet.put("pos", 0);
        packet.put("targetPos", 0);

        FtcDashboard dash = FtcDashboard.getInstance();
        dash.sendTelemetryPacket(packet);

        waitForStart();


        executeInstructions(new Instruction[]{
                new Wait((long) Math.pow(10, 9)),
                new DriveEasier(1000, 1000),
        });
    }
}
