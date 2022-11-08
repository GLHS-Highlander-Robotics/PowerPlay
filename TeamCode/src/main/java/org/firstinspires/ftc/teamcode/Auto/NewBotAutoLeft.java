package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Camera Auto Left Side")
public class NewBotAutoLeft extends LinearOpMode {

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
        robot.singleJointGripperArm.grab();
        //You need to have wait for start or else bad things happen
        waitForStart();

//        robot.singleJointGripperArm.ungrab();
//        robot.backTankDrive.drive(-400, -400, 0.5f);
//        robot.backTankDrive.drive(-520, 520, 0.5f);
//        robot.singleJointGripperArm.setArm(290);
//        robot.backTankDrive.drive(-200, -200, 0.25f);
//        robot.singleJointGripperArm.grab();
//        robot.backTankDrive.drive(520, 520, 0.25f);
//        robot.singleJointGripperArm.setArm(0);
//        robot.backTankDrive.drive(375, -375, 0.5f);
        switch(sleeveDetection.getPosition()) {
            case LEFT:
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(-3120, -3120, 0.5f);
                robot.singleJointGripperArm.setArm(450);
                robot.backTankDrive.drive(-500, 500, 0.5f);
                robot.backTankDrive.drive(-400, -400, 0.25f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(520, 520, 0.25f);
                robot.singleJointGripperArm.setArm(0);
                robot.backTankDrive.drive(490, -490, 0.5f);
                robot.backTankDrive.drive(3000, 3000, 0.5f);
                robot.backTankDrive.drive(650, -650, 0.5f);
                robot.singleJointGripperArm.setArm(89);
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(-4400, -4400, 0.5f);
                robot.backTankDrive.drive(-450, 450, 0.25f);
                break;
            case RIGHT:
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(-3120, -3120, 0.5f);
                robot.singleJointGripperArm.setArm(450);
                robot.backTankDrive.drive(-500, 500, 0.5f);
                robot.backTankDrive.drive(-400, -400, 0.25f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(520, 520, 0.25f);
                robot.singleJointGripperArm.setArm(0);
                robot.backTankDrive.drive(375, -375, 0.5f);
                robot.backTankDrive.drive(-1700, -1700, 0.5f);
                robot.backTankDrive.drive(-1000, 1000, 0.5f);
                robot.backTankDrive.drive(-1700, -1700, 0.5f);

                break;
            default:
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(-3120, -3120, 0.5f);
                robot.singleJointGripperArm.setArm(450);
                robot.backTankDrive.drive(-500, 500, 0.5f);
                robot.backTankDrive.drive(-400, -400, 0.25f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(520, 520, 0.25f);
                robot.singleJointGripperArm.setArm(0);
                robot.backTankDrive.drive(375, -375, 0.5f);
                robot.backTankDrive.drive(-1000, -1000, 0.5f);
                break;
        }

    }


}