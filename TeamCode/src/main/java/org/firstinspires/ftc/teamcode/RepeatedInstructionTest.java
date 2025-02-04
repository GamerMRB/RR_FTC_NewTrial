package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2.xy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.CombinedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Drive;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Log;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.RepeatedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SeriesInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmAngle;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmLength;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetClaw;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetTurn;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Wait;


@Autonomous
public class RepeatedInstructionTest extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();


        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        executeInstructions(new Instruction[]{
                new RepeatedInstruction(new SeriesInstruction(new Instruction[]{
                        new Wait((long) Math.pow(10, 9)),
                        new Log("WORDS!"),
                }), 10),
                new Log("Done"),
                new Wait((long) Math.pow(10, 9)),
        });
    }
}
