package org.firstinspires.ftc.teamcode.dynamicsLibrary.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class MovementRR extends Instruction{

    double x, y, tangent;
    public MovementRR(double x, double y, double tangent){
        this.x = x;
        this.y = y;
        this.tangent = tangent;
    }

    public void start(){
        Pose2d beginPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(x, y), tangent)
                .build()
        );
    }

    @Override
    public boolean update(UscOpMode opMode) {
        return !opMode.frontLeft.isBusy();
    }
}
