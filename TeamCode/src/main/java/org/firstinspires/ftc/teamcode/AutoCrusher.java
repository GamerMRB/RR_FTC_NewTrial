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


@Autonomous
public class AutoCrusher extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();


        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Instruction[] instructions = {
                //This is Project AlphaDelta. This executes 5 tasks with approximately 5 tasks within them to maintain readability. Good luck debugging.

                //First Sample & Retrieve Three Samples
                new SeriesInstruction(new Instruction[]{

                        new CombinedInstruction(new Instruction[]{
                                new Log("Attempt to hang first sample"),
                                new SetClaw(true),
                                new SetArmAngle(10),
                                new SetArmLength(25),
                                new Drive(xy(300, 0)),
                        }),

                        new CombinedInstruction(new Instruction[]{
//                                new SetArmAngle(-5),
                                new SetArmLength(1),
                                new SetClaw(false),
                                new Drive(xy(-300, 0)),
                        }),


                        new Drive(xy(0,850)),
                        new Drive(xy(760,0)),

                        //Sample Setup
                        new SeriesInstruction(new Instruction[]{
                                new Log("Attempting to push three samples"),
                                new SeriesInstruction(new Instruction[]{
                                        new Drive(xy(0,150)),
                                        new Drive(xy(-1372, 0)),
                                        new Drive(xy(1372, 0)),
                                }),

                                new SeriesInstruction(new Instruction[]{
                                        new Drive(xy(0,150)),
                                        new Drive(xy(-1372, 0)),
                                        new Drive(xy(1372, 0)),
                                }),

                                new SeriesInstruction(new Instruction[]{
                                        new Drive(xy(0,150)),
                                        new Drive(xy(-1372, 0)),
                                        new Drive(xy(610, -610)),
                                }),
                        }),
                }),


                //Second Sample
                new SeriesInstruction(new Instruction[]{
                        new CombinedInstruction(new Instruction[]{
                                new Log("Attempt to hang second sample"),
                                new SetTurn(Math.PI),
                                new SetArmAngle(-5),
                        }),

                        new Drive(xy(0, -610)),

                        new CombinedInstruction(new Instruction[]{
                                new SetClaw(true),
                                new SetArmAngle(6),
                                new SetTurn(Math.PI),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new SeriesInstruction(new Instruction[] {
                                        new Drive(xy(0, -1219)),
                                        new Drive(xy(457, 0)),
                                }),
                                new SetArmLength(10),
                                new SetArmAngle(-6),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Drive(xy(-150, 0)),
                                new SetClaw(false),
                                new SetArmLength(-10),
                        }),

                        new SeriesInstruction(new Instruction[]{
                                new Drive(xy(0, -1220)),
                                new Drive(xy(-600, 0)),
                        }),
                }),


                //Third sample
                new SeriesInstruction(new Instruction[]{
                        new CombinedInstruction(new Instruction[]{
                                new Log("Attempt to hang third sample"),
                                new SetTurn(Math.PI),
                                new SetArmAngle(-5),
                        }),


                        new CombinedInstruction(new Instruction[]{
                                new SetClaw(true),
                                new SetArmAngle(6),
                                new SetTurn(Math.PI),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new SeriesInstruction(new Instruction[] {
                                        new Drive(xy(0, -1219)),
                                        new Drive(xy(457, 0)),
                                }),
                                new SetArmLength(10),
                                new SetArmAngle(-6),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Drive(xy(-150, 0)),
                                new SetClaw(false),
                                new SetArmLength(-10),
                        }),

                        new SeriesInstruction(new Instruction[]{
                                new Drive(xy(0, -1220)),
                                new Drive(xy(-600, 0)),
                        }),
                }),


                //Fourth Sample
                new SeriesInstruction(new Instruction[]{
                        new CombinedInstruction(new Instruction[]{
                                new Log("Attempt to hang fourth sample"),
                                new SetTurn(Math.PI),
                                new SetArmAngle(-5),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new SetClaw(true),
                                new SetArmAngle(6),
                                new SetTurn(Math.PI),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new SeriesInstruction(new Instruction[] {
                                        new Drive(xy(0, -1219)),
                                        new Drive(xy(457, 0)),
                                }),
                                new SetArmLength(10),
                                new SetArmAngle(-6),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Drive(xy(-150, 0)),
                                new SetClaw(false),
                                new SetArmLength(-10),
                        }),

                        new SeriesInstruction(new Instruction[]{
                                new Drive(xy(0, -1220)),
                                new Drive(xy(-600, 0)),
                        }),
                }),

                //Fifth & Final Sample
                new SeriesInstruction(new Instruction[]{
                        new CombinedInstruction(new Instruction[]{
                                new Log("Attempt to hang fifth sample"),
                                new SetTurn(Math.PI),
                                new SetArmAngle(-5),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new SetClaw(true),
                                new SetArmAngle(6),
                                new SetTurn(Math.PI),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new SeriesInstruction(new Instruction[] {
                                        new Drive(xy(0, -1219)),
                                        new Drive(xy(457, 0)),
                                }),
                                new SetArmLength(10),
                                new SetArmAngle(-6),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Drive(xy(-150, 0)),
                                new SetClaw(false),
                                new SetArmLength(-10),
                        }),

                        //Grand Finale
                        new CombinedInstruction(new Instruction[]{
                                new Log("Attempt to self destruct"),
                                new SeriesInstruction(new Instruction[]{
                                        new Drive(xy(0, -1220)),
                                        new Drive(xy(-600, 0)),
                                }),
                        }),
                })

        };
        executeInstructions(instructions);
    }
}
