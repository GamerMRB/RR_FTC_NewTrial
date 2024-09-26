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


public abstract class UscOpMode extends LinearOpMode {

    protected DcMotorEx frontLeft;
    protected DcMotorEx frontRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx backRight;

    protected DcMotorEx armMotor1;
    protected DcMotorEx armMotor2;
    protected double posX;
    final double SAFETY_CLAW_SWING_ARM_HEIGHT = 900;
    protected boolean direction;
    protected boolean planeArmed;

    protected double posY;
    protected int currentArmPosition;



    protected static WebcamName camera1;
    protected static WebcamName camera2;
    protected static WebcamName camera3;
    protected static VisionPortal visionPortal;
    protected static VisionPortal visionPortal2;
    protected static VisionPortal visionPortal3;

    protected Servo clawServo1;
    protected Servo clawServo2;
    protected Servo clawRotation;
    //    protected Servo arrow;
    protected Servo planeLauncher;
    protected boolean aprilTagCam = false;

    protected final double WHEEL_DIAMETER = 96.0;
    protected final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected final double TICKS_PER_REVOLUTION = 538;
    protected final double SPEED_MAX = 1.0;
    protected final double STRAFE_SPEED = 0.75;
    protected final double SPEED_HALF = 0.5;
    protected final int ARM_SPEED = 2500;
    protected final double INTAKE_SPEED = 1.0d;
    protected final int MAX_ARM_HEIGHT = 2920;
    protected final int MIN_ARM_HEIGHT = 40;
    protected /*final*/ float servoPlacePosition;
    protected /*final*/ float servoGrabPosition;
    protected final double AIRPLANE_HOLD_POS = 0.7;
    protected final double AIRPLANE_RELEASE_POS = -1.0;
    protected final double ARM_TOLERANCE = 35;
    final double CLOSE_CLAW_1 = 0.3;
    final double CLOSE_CLAW_2 = 0.4;
    final double OPEN_CLAW_1 = 0.0;
    final double OPEN_CLAW_2 = 0.5;


    public void setUpHardware(boolean drivetrain, boolean cameras, boolean arm, boolean claw, boolean intake, boolean launch){
        if (drivetrain){
            setUpDrivetrain();
        }
        if(arm){
            setUpArm();
        }
        if(claw){
            setUpClaw();
        }
        if(launch){
            setUpAirplane();
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
    public void drivetrainDirection(boolean forward){
        if (forward){
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); // Motor 0
            backRight = hardwareMap.get(DcMotorEx.class, "backRight"); // Motor 1
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); // Motor 2
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); // Motor 3
            direction = true;
        }
        else {
            frontRight = hardwareMap.get(DcMotorEx.class, "backLeft"); // Motor 0
            frontLeft = hardwareMap.get(DcMotorEx.class, "backRight"); // Motor 1
            backRight = hardwareMap.get(DcMotorEx.class, "frontLeft"); // Motor 2
            backLeft = hardwareMap.get(DcMotorEx.class, "frontRight"); // Motor 3
            direction = false;
        }
    }

    public void setUpCameras(){
        aprilTagCam = true;
    }
    public void setUpArm(){
        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentArmPosition = ((armMotor1.getCurrentPosition() + armMotor2.getCurrentPosition())/2);
        armMotor1.setZeroPowerBehavior(BRAKE);
        armMotor2.setZeroPowerBehavior(BRAKE);
    }
    public void setUpClaw(){
        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo. class, "clawServo2");
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
    }

    public void setUpAirplane(){
        planeLauncher = hardwareMap.get(Servo.class, "launcher");
        planeLauncher.setPosition(AIRPLANE_HOLD_POS);
        planeArmed = false;
    }

    public void runOpMode() throws InterruptedException {
    }

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    protected void resetMotors() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    protected void moveArm(double distanceMm, double velocity) {
        double numberOfTicks = (distanceMm / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor1.setTargetPosition((int)numberOfTicks);
        armMotor2.setTargetPosition((int)numberOfTicks);

        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor1.setVelocity(velocity);
        armMotor2.setVelocity(velocity);;
        while (frontLeft.isBusy()) {
            telemetry.addData("velocity", frontLeft.getVelocity());
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("is at target", !frontLeft.isBusy());
            telemetry.update();
        }
    }
    protected void moveArmBack(double distanceMm, double velocity) {
        double numberOfTicks = (distanceMm / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor1.setTargetPosition((int)numberOfTicks);
        armMotor2.setTargetPosition((int)numberOfTicks);

        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor1.setVelocity(velocity);
        armMotor2.setVelocity(velocity);;
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

    // Visualization of scaling functions: https://www.desmos.com/calculator/attwysf9bd

    protected double scaleMovement(double vIn){
        return Math.pow(Math.sin((Math.PI * vIn) / 2), 3);
    }

    protected double scaleArmMovement(double vIn){
        return Math.pow(Math.E, -(Math.pow(vIn - (MAX_ARM_HEIGHT + MIN_ARM_HEIGHT)/2, 2)/ 1500000));
    }
}