package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2.xy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.CombinedInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Drive;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Log;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SeriesInstruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmAngle;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmLength;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetClaw;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetTurn;

import java.util.ArrayList;
import java.util.Arrays;
@Autonomous
public class AutoCrusher extends UscOpMode {
    public void runOpMode() {
        setUpHardware();
        waitForStart();

        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArrayList<Instruction> instructions = new ArrayList<>(Arrays.asList(
                //The Strat
                new CombinedInstruction(new Instruction[]{
                        new Log("Go to bar and score sample"),
                        new SetClaw(true),
                        new SetArmAngle(14.75),
                        new SetArmLength(7),
                }),

                new Drive(xy(700, 0)),

                new CombinedInstruction(new Instruction[]{
                        new Log("Reverse"),
                        new SetArmAngle(0),
                        new SetArmLength(0),
                        new SetClaw(false),
                        new Drive(xy(-300, 0)),
                }),

                /*
                    TODO: We should make the drive -300 incorporated into the drive 800 for the x.
                          We can do this by altering the drive 700x and arm angle/length
                */
                new Log("Get into pushing pos"),
                new Drive(xy(0, 850)),
                new Drive(xy(800, 0))
        ));

        //Sample Setup
        for (int i = 0; i <= 3; i++) {
            if (i == 3) {
                new SeriesInstruction(new Instruction[]{
                        new Log("Push sample" + i),
                        new Drive(xy(0, 300)),
                        new Drive(xy(-1372, 0)),
                        new Drive(xy(610, -610)),

                        new Log("Drive to the sample #2"),
                        new Drive(xy(-610, 0))
                });
            } else {
                instructions.addAll(Arrays.asList(
                        new Log("Push sample" + i),
                        new Drive(xy(0, 300)),
                        new Drive(xy(-1372, 0)),
                        new Drive(xy(1372, 0))
                ));
            }

        }

        //Second, Third, Fourth, Fifth Sample
        for (int i = 0; i <= 200; i += 50) {
            instructions.add(
                    new SeriesInstruction(new Instruction[]{
                            new CombinedInstruction(new Instruction[]{
                                    new Log("Position to grab specimen #" + i),
                                    new SetTurn(Math.PI),
                                    new SetArmAngle(5),
                            }),

                            new CombinedInstruction(new Instruction[]{
                                    new Log("Prepare for scoring specimen #" + i),
                                    new SetClaw(true),
                                    new SetArmAngle(14.75),
                                    new SetTurn(Math.PI),
                                    new SetArmLength(7),
                            }),

                            new CombinedInstruction(new Instruction[]{
                                    new Log("Drive and score specimen #" + i),
                                    new SeriesInstruction(new Instruction[]{
                                            new Drive(xy(0, -1219 - i)),
                                            new Drive(xy(457, 0)),
                                    }),
                            }),

                            new CombinedInstruction(new Instruction[]{
                                    new Log("Drive back and reset"),
                                    new SetClaw(false),
                                    new SetArmLength(0),
                                    new Drive(xy(0, -1220)),
                                    new Drive(xy(-750, 0)),
                            })
                    }));
        }

        Instruction[] array = instructions.toArray(new Instruction[0]);
        executeInstructions(array);
    }
}