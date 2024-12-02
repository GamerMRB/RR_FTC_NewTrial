package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;

@TeleOp
public class TagTrackTest extends UscOpMode {
    public void runOpMode() {
        setUpHardware();
        waitForStart();
        while(opModeIsActive()){
            telemetry.clear();
            updatePos();
            telemetry.addLine(robotPos.toString());
            telemetry.update();
        }
    }
}
