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
public class AutoCrusher extends UscOpMode {
    public void runOpMode() {
        setUpHardware();
        waitForStart();

        imu.resetYaw();
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArrayList<Instruction> instructions = new ArrayList<>(Arrays.asList(
            //The Strat
            new CombinedInstruction(new Instruction[]{
                new Log("Go to bar and score sample"),
                new SetClaw(true),
                new SetArmAngle(13),
                new SetArmLength(13),

                new SeriesInstruction(new Instruction[]{
                    new Wait(1000000000),
                    new DriveEasier(600, 0),
                })
            }),

            new CombinedInstruction(new Instruction[]{
                new SetClaw(false),
                new SetArmLength(0),

                new Log("Setup pushing"),
                new SetArmAngle(0),
                new SeriesInstruction(new Instruction[]{
                    new Wait(750000000),
                    new DriveEasier(0, -675),
                    new DriveEasier(675, 0)
                })
            })
        ));

        //Sample Setup
        for (int i = 0; i <= 2; i++) {
            if (i == 2) {
                instructions.add(
                    new SeriesInstruction(new Instruction[]{
                        new Log("Push sample" + (i + 1)),
                        new DriveEasier(0, -225),
                        new DriveEasier(-1050, 0),
                        new Log("Drive to the sample #2"),
                        new DriveEasier(0, 610),
                    })
                );
            }

            if ((i == 0) || (i == 1)) {
                instructions.add(
                    new SeriesInstruction(new Instruction[]{
                        new Log("Push sample" + (i + 1)),
                        new DriveEasier(0, -275),
                        new DriveEasier(-1100, 0),
                        new DriveEasier(1100, 0)
                    })
                );
            }
        }

        //Second, Third, Fourth, Fifth Sample
        for (int i = 0; i <= 3; i++) {
            instructions.add(
                new SeriesInstruction(new Instruction[]{
                    new SetTurn(Math.PI),
                    new CombinedInstruction(new Instruction[]{
                            new SetClaw(true),
                            new Log("Position to grab specimen # " + i),
                            new SetArmLength(0),
                            new SetArmAngle(6.5),
                    }),

                    new CombinedInstruction(new Instruction[]{
                            new Log("Prepare for scoring specimen # " + i),
                            new SetTurn(0),
                            new SetArmAngle(13),
                            new SetArmLength(13),
                            new Log("Drive and score specimen # " + i),
                            new SeriesInstruction(new Instruction[]{
                                new DriveEasier(457, 1000 + (i * 50)),
                            }),
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new Log("Drive back and reset for specimen# " + (i + 1)),
                        new SetClaw(false),
                        new DriveEasier(-500, -(1000 + (i * 50))),
                    })
                })
            );
        }

        executeInstructions(instructions.toArray(new Instruction[0]));
    }
}