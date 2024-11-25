package org.firstinspires.ftc.teamcode.dynamicsLibrary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;


public abstract class UscOpMode extends LinearOpMode {

    protected DcMotorEx frontLeft;
    protected DcMotorEx frontRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx backRight;
    protected DcMotorEx armPivot;
    protected DcMotorEx armSlide;

    protected static Camera[] cameras = {
    };
    protected WebcamName camera1;
    protected WebcamName camera2;
    protected WebcamName camera3;
    protected VisionPortal visionPortal;
    protected VisionPortal visionPortal2;
    protected VisionPortal visionPortal3;

    protected Position robotPos;
    protected Vec3 clawPos;
    protected double armAngle;
    protected double armLength;
    protected PID2 movementPID = new PID2(1, 0, 0);
    protected PID rotationPID = new PID(1, 0, 0);

    protected final double WHEEL_DIAMETER = 96.0;
    protected final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected final double TICKS_PER_REVOLUTION = 538;
    protected final double SPEED_MAX = 1.0;
    protected final double STRAFE_SPEED = 0.75;
    protected final Vec3 pivotPos = Vec3.xyz(0, 0, 13.25);
    protected final double INITIAL_ARM_ANGLE = - Math.PI/4;
    protected final double MIN_ARM_LENGTH = 10.375;


    public void setUpHardware(){
        setUpDrivetrain();
        //setUpCameras();
        setUpArm();
    }

    public void setUpCameras(){
        cameras = new Camera[]{
                Camera.makeIt(Position.xyr(0, 3, 0), hardwareMap.get(WebcamName.class, "Webcam 1")),
//                Camera.makeIt(Position.xyr(0, -3, Math.PI), hardwareMap.get(WebcamName.class, "Webcam 2")),
        };
    }
    public void updatePos(){
        Vec2 pos = Vec2.zero;
        Vec2 dir = Vec2.zero;
        long tagCount = 0;
        for(Camera camera : cameras){
            ArrayList<AprilTagDetection> detections = camera.processor.getDetections();
            for(AprilTagDetection detection : detections){
                Position tag = Position.xyr(detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1), 0);
                Position cam = tag.remove(Position.xyr(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw));
                Position robot = cam.remove(camera.pos);
                pos = pos.add(robot.disp);
                dir = dir.add(robot.rot);
            }
            tagCount += detections.size();
        }
        robotPos = Position.vr(pos.div(tagCount), dir.angle());
    }

    protected void setUpDirections(){
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void setUpDrivetrain() {
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); // Motor 0
        backRight = hardwareMap.get(DcMotorEx.class, "backRight"); // Motor 2
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); // Motor 1
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); // Motor 4
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        setUpDirections();
    }
    public void setUpArm(){
        armAngle = INITIAL_ARM_ANGLE;
        armLength = MIN_ARM_LENGTH;
        armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        armPivot.setZeroPowerBehavior(BRAKE);
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

    protected void setPower(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
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


    protected void pow(double forward, double side, double rot){
        double maxPow = Math.max(1, Math.abs(forward) + Math.abs(side) + Math.abs(forward));
        frontLeft.setPower((forward - side - rot) / maxPow);
        frontRight.setPower((forward + side + rot) / maxPow);
        backLeft.setPower((forward + side - rot) / maxPow);
        backRight.setPower((forward - side + rot) / maxPow);
    }
    protected void pow(Position position){
        pow(position.disp.y, position.disp.x, position.rot);
    }
    protected void pow(Vec2 disp, double rot){
        pow(disp.y, disp.x, rot);
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
    protected double mod(double a, double b){
        return a - b*Math.floor(a / b);
    }
    protected double simplifyAngle(double angle){
        return mod(angle + Math.PI, 2*Math.PI) - Math.PI;
    }
    protected void moveTo(Position position){
        Position diff = robotPos.difference(position);
        double posAllowance = 1.5;
        double angAllowance = Math.PI/8;
        double then = (double) System.currentTimeMillis() / 1000;
        while(
                diff.disp.mag() >= posAllowance ||
                Math.abs(simplifyAngle(diff.rot)) >= angAllowance
        ){
            double now = getRuntime();
            double dt = now - then;
            movementPID.update(diff.disp, dt);
            rotationPID.update(simplifyAngle(diff.rot), dt);
            pow(movementPID.getPow(), rotationPID.getPow());
            updatePos();
            diff = robotPos.difference(position);
            then = now;
        }
    }
//    protected void moveTo(Vec2 target){
//        Vec2 diff = target.sub(robotPos.disp).unit();
//        double angle;
//        double sin;
//        double cos;
//        while (diff.mag() >= 1.5) {
//            diff = target.sub(robotPos.disp).unit();
//            angle = diff.angle(robotPos.rot);
//            sin = Math.sin(angle);
//            cos = Math.cos(angle);
//            frontLeft.setPower(sin + cos);
//            frontRight.setPower(cos - sin);
//            backLeft.setPower(cos - sin);
//            backRight.setPower(sin + cos);
//            updatePos();
//        }
//        setPower(0);
//    }

    protected void calculateClaw(){
        clawPos = pivotPos.add(Vec3.v2z(robotPos.dir().mult(armLength * Math.cos(armAngle)) , armLength * Math.sin(armAngle)));
    }
    protected void moveClawTo(Vec3 target){
    // needs movement code first
    }


    // Visualization of scaling functions: https://www.desmos.com/calculator/attwysf9bd

    protected double scaleMovement(double vIn){
        return Math.pow(Math.sin((Math.PI * vIn) / 2), 3);
    }

}