package org.firstinspires.ftc.teamcode.dynamicsLibrary;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class Camera {
    public final Position pos;
    public final WebcamName webcam;
    public final VisionPortal portal;
    public final AprilTagProcessor processor;
    private Camera (Position posIn, WebcamName webcamIn, AprilTagProcessor processorIn, VisionPortal portalIn){
        pos = posIn;
        webcam = webcamIn;
        processor = processorIn;
        portal = portalIn;
    }
    public static Camera makeIt(Position posIn, WebcamName webcamName, int id){
        VisionPortal.Builder builder = new VisionPortal.Builder();

        AprilTagProcessor processor = AprilTagProcessor.easyCreateWithDefaults();
        builder.addProcessor(processor);

        builder.setLiveViewContainerId(id);

        builder.setCamera(webcamName);

        return new Camera(posIn, webcamName, processor, builder.build());
    }
    public static Camera makeIt(Position posIn, WebcamName webcamName){
        return Camera.makeIt(posIn, webcamName, 1);
    }
}

