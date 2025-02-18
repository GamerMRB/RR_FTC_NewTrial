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
//        setVelocityPIDFCoefficients(1.175,0.1175,0, 11.75);

        ArrayList<Instruction> instructions = new ArrayList<>(Arrays.asList(
            //First Sample
            new CombinedInstruction(new Instruction[]{
                new Log("Go to bar and score sample"),
                new SetClaw(true),
                new SetArmAngle(13),
                new SetArmLength(13),

                new SeriesInstruction(new Instruction[]{
                    new Wait(1000000000),
                    new DriveEasier(650, 0),
                })
            }),

            new CombinedInstruction(new Instruction[]{
                new SetClaw(false),
                new SetArmLength(0),

                new Log("Setup pushing"),
                new SetArmAngle(0),
                new SeriesInstruction(new Instruction[]{
                    new Wait(750000000),
                    new DriveEasier(0, -900),
                    new DriveEasier(675, 0)
                })
            }),

            new SeriesInstruction(new Instruction[]{
                new Log("Push sample " + 1),
                new DriveEasier(0, -250),
                new DriveEasier(-1200, 0),
                new DriveEasier(1200, 0)
            }),

            new SeriesInstruction(new Instruction[]{
                new Log("Push sample " + 2),
                new DriveEasier(0, -275),
                new DriveEasier(-1300, 0),
                new Log("Drive to the specimen"),
                new DriveEasier(0, 475),
            })
        ));

        //Second, Third, and Fourth Sample
        for (int i = 0; i <= 2; i++) {
            if (i == 0) {
                instructions.add(
                        new SeriesInstruction(new Instruction[]{
                                new SetTurn(Math.PI / 2),
                                new SetTurn(Math.PI),
                                new CombinedInstruction(new Instruction[]{
                                        new Log("Position to grab specimen # " + i),
                                        new SetArmAngle(6.5),
                                        new Wait((long) ((long) 5 * Math.pow(10, 8)))
                                }),

                                new SetClaw(true),
                                new SetArmAngle(14),
                                new SetTurn(-Math.PI / 2),
                                new SetTurn(0),
                                new DriveEasier(0,100),

                                new CombinedInstruction(new Instruction[]{
                                        new Log("Prepare for scoring specimen # " + i),
                                        new SetArmLength(10),
                                        new Log("Drive and score specimen # " + i),
                                        new DriveEasier(600, 1350 + (i * 10))
                                }),

                                new CombinedInstruction(new Instruction[]{
                                        new Log("Drive back and reset for specimen# " + (i + 1)),
                                        new SetClaw(false),
                                        new SetArmLength(3),
                                        new DriveEasier(-600, -1 * (1375 + (i * 10))),
                                }),

                        })
                );
            }

            if (i == 1 | i == 2){
                instructions.add(
                        new SeriesInstruction(new Instruction[]{
                                new SetTurn(Math.PI / 2),
                                new SetTurn(Math.PI),
                                new CombinedInstruction(new Instruction[]{
                                        new Log("Position to grab specimen # " + i),
                                        new SetArmAngle(6.5),
                                        new Wait((long) ((long) 5 * Math.pow(10, 8)))
                                }),

                                new CombinedInstruction(new Instruction[]{
                                        new SetClaw(true),
                                        new SetArmAngle(14),
                                        new DriveEasier(0,i * 600),
                                }),


                                new SetTurn(-Math.PI / 2),
                                new SetTurn(0),
                                new SetArmLength(10),

                                new DriveEasier(600,0),
                                new DriveEasier(-600,0),

                                new CombinedInstruction(new Instruction[]{
                                        new Log("Drive back and reset for specimen# " + (i + 1)),
                                        new SetClaw(false),
                                        new SetArmLength(3),
                                }),

                                new DriveEasier(0, -i * 600)
                        })
                );
            }

        }

        executeInstructions(instructions.toArray(new Instruction[0]));
    }
}