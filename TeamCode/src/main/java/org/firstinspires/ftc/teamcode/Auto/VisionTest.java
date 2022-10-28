package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Signal Sleeve Test")
public class VisionTest extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }


        OldRobot robot = new OldRobot(this, hardwareMap, telemetry, gamepad1);

        //Put motors in encoder mode
        robot.backTankDrive.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backTankDrive.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        robot.singleJointGripperArm.grab();
        robot.backTankDrive.drive(200, 200, 0.25f);
        robot.backTankDrive.drive(500, -500, 0.25f);
        robot.singleJointGripperArm.setArm(430);
        robot.backTankDrive.drive(50, 50, 0.10f);
        robot.singleJointGripperArm.ungrab();
        robot.backTankDrive.drive(-50, -50, 0.10f);
        robot.singleJointGripperArm.setArm(0);
        robot.backTankDrive.drive(-500, 500, 0.25f);



        switch(sleeveDetection.getPosition()) {
            case LEFT:
                robot.backTankDrive.drive(-500, 500, 0.25f);
                robot.backTankDrive.drive(800, 800, 0.25f);
                robot.backTankDrive.drive(500, -500, 0.25f);
                robot.backTankDrive.drive(800, 800, 0.25f);
                break;
            case RIGHT:
                robot.backTankDrive.drive(1000, 100, 0.25f);
                robot.backTankDrive.drive(1000, -1000, 0.25f);
                robot.backTankDrive.drive(-1000, 1000, 0.25f);
                break;

            case CENTER:
                robot.backTankDrive.drive(1200, 1200, 0.25f);
                break;
        }

    }
}