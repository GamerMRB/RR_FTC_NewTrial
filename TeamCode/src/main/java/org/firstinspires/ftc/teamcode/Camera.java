package org.firstinspires.ftc.teamcode;

import android.webkit.WebChromeClient;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import java.util.ArrayList;


public class Camera {
    public final Position pos;
    public final WebcamName webcam;
    public final VisionPortal portal;
    public Camera (Position posIn, VisionPortal portalIn, WebcamName webcamIn){
        pos = posIn;
        portal = portalIn;
        webcam = webcamIn;
    }
    static Camera MAKEIT(Position posIn, WebcamName webcamName, VisionProcessor[] visionProcessors){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        for(VisionProcessor visionProcessor : visionProcessors){
            builder.addProcessor(visionProcessor);
        }
        return new Camera(posIn, builder.build(), webcamName);
    }


}
