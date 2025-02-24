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
            //First Sample
                new SetClaw(false),
                new CombinedInstruction(new Instruction[]{
                    new SetArmLength(3),
                    new SeriesInstruction(new Instruction[]{
                        new Wait(200000000),
                        new Log("Go to bar and score sample"),
                        new SetArmAngle(15.5),
                        new SetArmLength(14),
                        new SeriesInstruction(new Instruction[]{
                            new Wait(1500000000),
                            new DriveEasier(660, 0),
                        })
                    }),
                }),

            new CombinedInstruction(new Instruction[]{
                new SetClaw(true),
                new SetArmLength(0),

                new Log("Setup pushing"),
                new SetArmAngle(0),
                new SeriesInstruction(new Instruction[]{
                    new Wait(750000000),
                    new DriveEasier(0, -695),
                    new DriveEasier(640, 0)
                })
            }),

            new SeriesInstruction(new Instruction[]{
                new Log("Push sample " + 1),
                new DriveEasier(0, -225),
                new DriveEasier(-1100, 0),
                new DriveEasier(1000,0)
            }),

            new SeriesInstruction(new Instruction[]{
                new Log("Push sample " + 2),
                new DriveEasier(0, -250),
                new DriveEasier(-1090, 0),
                new Log("Drive to the specimen"),
                new DriveEasier(0, 300),
            })
        ));

        //Second, Third, and Fourth Sample
        for (int i = 0; i <= 1; i++) {
            if (i == 0) {
                instructions.add(
                    new SeriesInstruction(new Instruction[]{
                        new SetTurn(Math.PI),
                        new CombinedInstruction(new Instruction[]{
                                new Log("Position to grab specimen # " + i),
                                new SetArmAngle(6.5),
                                new Wait((long) ((long) 5 * Math.pow(10, 8)))
                        }),

                        new SetClaw(false),
                        new SetArmAngle(14),
                        new SetTurn(0),

                        new CombinedInstruction(new Instruction[]{
                                new Log("Prepare for scoring specimen # " + i),
                                new SetArmLength(15),
                                new Log("Drive and score specimen # " + i),
                                new DriveEasier(585, 1250)
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Log("Drive back and reset for specimen# " + (i + 1)),
                                new SetClaw(true),
                                new SetArmLength(0),
                                new DriveEasier(-565, -1280),
                        }),
                    })
                );
            }

            if (i == 1){
                instructions.add(
                    new SeriesInstruction(new Instruction[]{
                        new SetTurn(Math.PI),
                        new CombinedInstruction(new Instruction[]{
                                new Log("Position to grab specimen # " + i),
                                new SetArmAngle(7),
                                new Wait((long) ((long) 5 * Math.pow(10, 8)))
                        }),

                        new SetClaw(false),
                        new SetArmAngle(14),
                        new SetTurn(0),

                        new CombinedInstruction(new Instruction[]{
                            new SetArmLength(20),
                            new DriveEasier(385,1075),
                        }),

                        new DriveEasier(180,0),

                        new CombinedInstruction(new Instruction[]{
                            new Log("Drive back and reset for specimen# " + (i + 1)),
                            new SetClaw(true),
                            new SetArmLength(0),
                            new DriveEasier(-475,-((i * 100) + 750))
                        }),
                        new DriveEasier(-40,0)
                    })
                );
            }
        }

        executeInstructions(instructions.toArray(new Instruction[0]));
    }
}