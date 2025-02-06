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

@SuppressWarnings("unused")
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
                new Log("Setup pushing"),
                new SetArmAngle(0),
                new SetArmLength(0),
                new SetClaw(false),
                new SeriesInstruction(new Instruction[]{
                    new Drive(xy(0, 850)),
                    new Drive(xy(500, 0))
                })
            })
        ));

        //Sample Setup
        for (int i = 0; i <= 2; i++) {
            if (i == 2) {
                instructions.add(
                    new SeriesInstruction(new Instruction[]{
                        new Log("Push sample" + (i + 1)),
                        new Drive(xy(0, 300)),
                        new Drive(xy(-1372, 0)),
                        new Drive(xy(610, -610)),

                        new Log("Drive to the sample #2"),
                        new Drive(xy(-610, 0))
                    })
                );
            } else {
                instructions.add(
                    new SeriesInstruction(new Instruction[]{
                        new Log("Push sample" + (i + 1)),
                        new Drive(xy(0, 300)),
                        new Drive(xy(-1372, 0)),
                        new Drive(xy(1372, 0))
                    })
                );
            }

        }

        //Second, Third, Fourth, Fifth Sample
        for (int i = 0; i <= 3; i ++) {
            instructions.add(
                new SeriesInstruction(new Instruction[]{
                    new CombinedInstruction(new Instruction[]{
                        new Log("Position to grab specimen #" + (i + 2)),
                        new SetTurn(Math.PI),
                        new SetArmAngle(5),
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new Log("Prepare for scoring specimen #" + (i + 2)),
                        new SetClaw(true),
                        new SetArmAngle(14.75),
                        new SetTurn(Math.PI),
                        new SetArmLength(7),
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new Log("Drive and score specimen #" + (i + 2)),
                        new SeriesInstruction(new Instruction[]{
                                new Drive(xy(0, -1219 - (i * 50))),
                                new Drive(xy(457, 0)),
                        }),
                    }),

                    new CombinedInstruction(new Instruction[]{
                        new Log("Drive back and reset"),
                        new SetClaw(false),
                        new SetArmLength(0),
                        new Drive(xy(0, -1220 - (i * 50))),
                        new Drive(xy(-750, 0)),
                    })
                })
            );
        }

        executeInstructions(instructions.toArray(new Instruction[0]));
    }
}