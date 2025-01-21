package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.dynamicsLibrary.UscOpMode;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.Vec2;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Drive;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Instruction;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.MoveArm;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmAngle;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.SetArmLength;
import org.firstinspires.ftc.teamcode.dynamicsLibrary.auto.Wait;

import java.util.HashMap;
import java.util.Map;

@Autonomous
public class AutoCrusher extends UscOpMode {

    public void runOpMode() {
        setUpHardware();
        waitForStart();

        Instruction[] instructions = {
                new Drive(Vec2.xy(-718,0)),
//                new MoveArm(Vec2.polar(100, 1)),
//                new SetArmAngle(2.7),
//                new SetArmLength(48),
//                new Wait((long) (4*Math.pow(10, 9))),
//                new MoveArm(Vec2.polar(0, 0)),
        };
        executeInstructions(instructions);
    }
}
