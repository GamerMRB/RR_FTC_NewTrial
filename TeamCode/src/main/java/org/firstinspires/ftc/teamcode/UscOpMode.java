package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;


public abstract class UscOpMode extends LinearOpMode {

    protected DcMotorEx frontLeft;
    protected DcMotorEx frontRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx backRight;

    protected static WebcamName camera1;
    protected static WebcamName camera2;
    protected static WebcamName camera3;
    protected static VisionPortal visionPortal;
    protected static VisionPortal visionPortal2;
    protected static VisionPortal visionPortal3;

    protected Vec2 robotPos;
    protected Vec2 robotDirection;
    protected Vec3 clawPos;
    protected double armAngle;
    protected double armLength;

    protected final double WHEEL_DIAMETER = 96.0;
    protected final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected final double TICKS_PER_REVOLUTION = 538;
    protected final double SPEED_MAX = 1.0;
    protected final double STRAFE_SPEED = 0.75;
    protected final Vec3 pivotPos = Vec3.xyz(0, 0, 13.25);
    protected final double INITIAL_ARM_ANGLE = - Math.PI/4;
    protected final double MIN_ARM_LENGTH = 10.375;

    public void setUpHardware(boolean drivetrain, boolean cameras, boolean arm, boolean claw, boolean intake, boolean launch){
        if (drivetrain){
            setUpDrivetrain();
        }
    }
    public void setUpDrivetrain() {
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); // Motor 3
        backRight = hardwareMap.get(DcMotorEx.class, "backRight"); // Motor 2
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); // Motor 1
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); // Motor 0
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);

    }
    public void setUpArm(){
        armAngle = INITIAL_ARM_ANGLE;
        armLength = MIN_ARM_LENGTH;

    }

    public void runOpMode() throws InterruptedException {
    }

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    protected void resetMotors() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    protected void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void setVelocity(double velocity) {
        frontLeft.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }

    protected void setTargetPosition(int numberOfTicks) {
        frontLeft.setTargetPosition(numberOfTicks);
        frontRight.setTargetPosition(numberOfTicks);
        backLeft.setTargetPosition(numberOfTicks);
        backRight.setTargetPosition(numberOfTicks);
    }

    protected void motorsForward() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    protected void motorsBackward() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    protected void motorsStrafeLeft() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD); //+
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); //-
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD); //+
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); //-
    }

    protected void motorsStrafeRight() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); //-
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD); //+
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE); //-
        backRight.setDirection(DcMotorSimple.Direction.FORWARD); //+
    }

    protected void motorsLeft() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    protected void motorsRight() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    protected void movementPowerDisable() {
        frontLeft.setMotorDisable();
        frontRight.setMotorDisable();
        backLeft.setMotorDisable();
        backRight.setMotorDisable();
    }

    protected void movementPowerEnable() {
        frontLeft.setMotorEnable();
        frontRight.setMotorEnable();
        backLeft.setMotorEnable();
        backRight.setMotorEnable();
    }

    protected void turnLeft(double degrees, double velocity) {
        double numberOfTicks = degrees * ((TICKS_PER_REVOLUTION * 2) / 90);
        resetMotors();
        motorsLeft();
        setTargetPosition((int) numberOfTicks);
        setRunToPosition();
        setVelocity(velocity);
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }

    protected void turnRight(double degrees, double velocity) {
        double numberOfTicks = degrees * ((TICKS_PER_REVOLUTION * 2) / 90);
        resetMotors();
        motorsRight();
        setTargetPosition((int) numberOfTicks);
        setRunToPosition();
        setVelocity(velocity);
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }

    protected void moveForward(double distanceMm, double velocity) {
        double numberOfTicks = (distanceMm / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
        resetMotors();
        motorsForward();
        setTargetPosition((int) numberOfTicks);
        setRunToPosition();
        setVelocity(velocity);
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }

    protected void moveBackward(double distanceMm, double velocity) {
        double numberOfTicks = (distanceMm / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
        resetMotors();
        motorsBackward ();
        setTargetPosition((int) numberOfTicks);
        setRunToPosition();
        setVelocity(velocity);
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }


    protected void strafeLeft(double distanceMm, double velocity) {
        double numberOfTicks = (distanceMm / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
        resetMotors();
        motorsStrafeLeft();
        setTargetPosition((int) numberOfTicks);
        setRunToPosition();
        setVelocity(velocity);
        if (frontLeft.getCurrentPosition() >= numberOfTicks) {
            setVelocity(0.0);
        }
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }

    protected void strafeRight(double distanceMm, double velocity) {
        double numberOfTicks = (distanceMm / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
        resetMotors();
        motorsStrafeRight();
        setTargetPosition((int) numberOfTicks);
        setRunToPosition();
        setVelocity(velocity);
        if (frontLeft.getCurrentPosition() >= numberOfTicks) {
            setVelocity(0.0);
        }
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }

    protected void calculateClaw(){
        clawPos = pivotPos.add(Vec3.v2z(robotDirection.mult(armLength * Math.cos(armAngle)) , armLength * Math.sin(armAngle)));
    }
    protected void moveClawTo(Vec3 target){

    }

    // Visualization of scaling functions: https://www.desmos.com/calculator/attwysf9bd

    protected double scaleMovement(double vIn){
        return Math.pow(Math.sin((Math.PI * vIn) / 2), 3);
    }

}