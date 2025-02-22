package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.CombinedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.DriveEasier;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SeriesInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmAngle;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmLength;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetClaw;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetTurn;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Wait;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@Autonomous
public class AutoCrusher2_0 extends UscOpMode {
    public void runOpMode() {
        setUpHardware();
        waitForStart();

        imu.resetYaw();
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArrayList<Instruction> instructions = new ArrayList<>(List.of(
            new CombinedInstruction(new Instruction[]{
                new DriveEasier(1230, -400),
                new SetTurn(Math.PI),
                new SeriesInstruction(new Instruction[]{
                    new Wait((long) (2 * Math.pow(10, 9))),
                    new DriveEasier(-1150, -50),
                })
            })
        ));

        for (int i = 0; i <= 2; i++) {
            instructions.add(
                new SeriesInstruction(new Instruction[]{
                    new CombinedInstruction(new Instruction[]{
                        new SetArmAngle(6.5),
                        new SeriesInstruction(new Instruction[]{
                            new Wait((long) (5 * Math.pow(10, 8))),
                            new SetClaw(false),
                            new SetArmAngle(14),
                        })
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new SetArmLength(16),
                        new DriveEasier(602,1000),
                        new SetTurn(0),
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new SetClaw(true),
                        new SetArmLength(0),
                        new DriveEasier(-602, -1020),
                        new SetTurn(Math.PI)
                    })
                })
            );
        }

        executeInstructions(instructions.toArray(new Instruction[0]));
    }
}