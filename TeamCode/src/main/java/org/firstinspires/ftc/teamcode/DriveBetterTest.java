package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2.xy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.CombinedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Drive;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.DriveBetter;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.LinearPath;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Log;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.RepeatedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SeriesInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmAngle;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmLength;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetClaw;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetTurn;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Wait;


@Autonomous
public class DriveBetterTest extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();


        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetIMU();


        executeInstructions(new Instruction[]{
//                new Wait((long) Math.pow(10, 9)),
                new DriveBetter(
                    new LinearPath(Vec2.xy(0, 2000), 200, 2000)
                )
        });
    }
}
