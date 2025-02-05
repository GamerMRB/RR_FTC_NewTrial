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


@Autonomous
public class AutoCrusher extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();

        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RepeatedInstruction twoX = new RepeatedInstruction(
                new SeriesInstruction(new Instruction[] {
                    new Log("Push first sample"),
                    new Drive(xy(0,150)),
                    new Drive(xy(-1372, 0)),
                    new Drive(xy(1372, 0)),
                }
        ), 2);

        RepeatedInstruction bulk = new RepeatedInstruction(
                new SeriesInstruction(new Instruction[]{
                        new CombinedInstruction(new Instruction[]{
                                new Log("Position to grab specimen"),
                                new SetTurn(Math.PI),
                                new SetArmAngle(5),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Log("Prepare for scoring"),
                                new SetClaw(true),
                                new SetArmAngle(11),
                                new SetTurn(Math.PI),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Log("Drive and score the specimen"),
                                new SeriesInstruction(new Instruction[] {
                                        new Drive(xy(0, -1219)),
                                        new Drive(xy(457, 0)),
                                }),
                                new SetArmLength(50),
                                new SetArmAngle(5),
                        }),

                        new CombinedInstruction(new Instruction[]{
                                new Log("Drive back and reset"),
                                new Drive(xy(-150, 0)),
                                new SetClaw(false),
                                new SetArmLength(0),
                        }),

                        new SeriesInstruction(new Instruction[]{
                                new Log("Set up for grabbing next specimen"),
                                new Drive(xy(0, -1220)),
                                new Drive(xy(-600, 0)),
                        }),
                })
                ,4);

        Instruction[] instructions = {
                //This is Project AlphaDelta. This executes 5 tasks with approximately 5 tasks within them to maintain readability. Good luck debugging.

                //First Sample & Retrieve Three Samples
                new SeriesInstruction(new Instruction[]{

                        new CombinedInstruction(new Instruction[]{
                                new Log("Go to bar"),
                                new SetClaw(true),
                                new SetArmAngle(14.75),
                                new SetArmLength(7),
                        }),

                        new Drive(xy(700, 0)),

                        new CombinedInstruction(new Instruction[]{
                                new Log("Score sample"),
                                new SetArmAngle(0),
                                new SetArmLength(0),
                                new SetClaw(false),
                                new Drive(xy(-300, 0)),
                        }),

                        new Log("Get into pushing pos"),
                        new Drive(xy(0,900)),
                        new Drive(xy(800,0)),

                        //Sample Setup
                        new Log("Push two samples"),
                        twoX,

                        new SeriesInstruction(new Instruction[]{
                                new Log("Push third sample"),
                                new Drive(xy(0,150)),
                                new Drive(xy(-1372, 0)),
                                new Drive(xy(610, -610)),
                        }),

                }),

                //Setup for bulk
                new Log("Drive to the sample x2"),
                new Drive(xy(610, 0)),

                //Second, Third, Fourth, Fifth Sample
                bulk
        };

        executeInstructions(instructions);
    }
}
