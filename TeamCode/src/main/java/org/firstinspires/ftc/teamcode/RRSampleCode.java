package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RRSampleCode extends UscOpMode {
    public void runOpMode(){
        Pose2d beginPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(50,50), Math.PI/4)
                .build()
        );


    }
}
