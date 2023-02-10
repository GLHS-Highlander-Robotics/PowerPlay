package org.firstinspires.ftc.teamcode.old.subsystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.old.RobotOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Webcam extends Subsystem {
    public OpenCvCamera camera;
    public final OpenCvPipeline pipeline;
    public final String name;
    public final static int WIDTH = 320;
    public final static int HEIGHT = 240;

    public Webcam(RobotOpMode opMode, String name, OpenCvPipeline pipeline) {
        super(opMode);
        this.name = name;
        this.pipeline = pipeline;
    }

    @Override
    public void setup() {
        int monitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, name), monitorViewId);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
}
