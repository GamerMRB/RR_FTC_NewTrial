package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.CombinedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.DriveEasier;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Log;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SeriesInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmAngle;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmLength;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetClaw;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetTurn;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Wait;

import java.util.ArrayList;
import java.util.Arrays;

@SuppressWarnings("unused")
@Autonomous
public class AutoCrusher2_0 extends UscOpMode {
    public void runOpMode() {
        setUpHardware();
        waitForStart();

        imu.resetYaw();
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArrayList<Instruction> instructions = new ArrayList<>(Arrays.asList(//First Sample
            new CombinedInstruction(new Instruction[]{
                new SetClaw(false),
                new Log("Go to bar and score sample"),
                new SetClaw(false),
                new SetArmAngle(13),
                new SetArmLength(16),

                new SeriesInstruction(new Instruction[]{
                    new Wait(1000000000),
                    new DriveEasier(625, 0),
                })
            }),

            new CombinedInstruction(new Instruction[]{
                new SetClaw(true),
                new SetArmLength(0),

                new Log("Setup pushing"),
                new SetArmAngle(0),
                new SeriesInstruction(new Instruction[]{
                    new Wait(750000000),
                    new DriveEasier(0, -680),
                    new DriveEasier(675, 0)
                })
            }),

            new SeriesInstruction(new Instruction[]{
                new Log("Push sample 1"),
                new DriveEasier(0, -225),
                new DriveEasier(-1100, 0),
                new DriveEasier(1100, 0)
            }),

            new SeriesInstruction(new Instruction[]{
                new Log("Push sample 2"),
                new DriveEasier(0, -250),
                new DriveEasier(-1125, 0),
                new Log("Drive to the specimen"),
                new CombinedInstruction(new Instruction[]{
                    new DriveEasier(0, 420),
                    new SetTurn(Math.PI),
                })
            }),

            //Second Sample
            new SeriesInstruction(new Instruction[]{
                new CombinedInstruction(new Instruction[]{
                    new Log("Position to grab specimen #2"),
                    new SetArmAngle(6.5),
                    new Wait((long) ((long) 5 * Math.pow(10, 8)))
                }),

                new SetClaw(false),
                new SetArmAngle(15),

                new CombinedInstruction(new Instruction[]{
                    new Log("Prepare for scoring specimen #2"),
                    new SetArmLength(13),
                    new Log("Drive and score specimen #2"),
                    new DriveEasier(575, 1050),
                    new SetTurn(0),
                }),

                new CombinedInstruction(new Instruction[]{
                    new Log("Drive back and reset for specimen #3"),
                    new SetClaw(true),
                    new SetArmLength(6),
                    new DriveEasier(-589, -1000),
                    new SetTurn(Math.PI),
                })
            })
        ));

        //Third and Fourth Sample
        for (int i = 1; i <= 2; i++) {
            instructions.add(
                new SeriesInstruction(new Instruction[]{
                    new CombinedInstruction(new Instruction[]{
                        new Log("Position to grab specimen # " + (i + 2)),
                        new SetArmAngle(6.5),
                        new Wait((long) ((long) 5 * Math.pow(10, 8)))
                    }),

                    new SetClaw(false),
                    new SetArmAngle(((i - 1) * .5) + 14),

                    new CombinedInstruction(new Instruction[]{
                        new SetArmLength(16 - (i - 1)),
                        new DriveEasier(435, (i * 100) + 750),
                        new SetTurn(0),
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new Log("Drive back and reset for specimen# " + (i + 3)),
                        new SetClaw(true),
                        new SetArmLength(6),
                        new DriveEasier(-510, -((i * 100) + 750)),
                        new SetTurn(Math.PI)
                    })
                })
            );
        }

        executeInstructions(instructions.toArray(new Instruction[0]));
    }
}