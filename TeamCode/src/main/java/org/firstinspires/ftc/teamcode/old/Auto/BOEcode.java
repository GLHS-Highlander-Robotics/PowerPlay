package org.firstinspires.ftc.teamcode.old.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.old.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.old.Subsystems.BoeDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BOE")
public class BOEcode extends LinearOpMode {

    BoeDetection coneDetection = new BoeDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        coneDetection = new BoeDetection();
        camera.setPipeline(coneDetection);

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
            telemetry.addData("Cone: ", coneDetection.getPosition());
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
        int straight = 0;
        boolean turningRight = true;
        while (coneDetection.getPosition() == BoeDetection.Cone.UNDECIDED || coneDetection.getPosition() == BoeDetection.Cone.RED) {

            if(turningRight){
                robot.backTankDrive.drive(-10, 10, 0.5f);
                straight+=10;
            }else{
                robot.backTankDrive.drive(10, -10, 0.5f);
                straight-=10;
            }
            if(straight>=500){
                turningRight=false;
            }else if(straight<=-500){
                turningRight=true;
            }
        }

        robot.backTankDrive.backLeftMotor.setPower(0);
        robot.backTankDrive.backRightMotor.setPower(0);

        switch(coneDetection.getPosition()) {
            case RED:
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(-3120, -3120, 0.5f);
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(straight, -straight, 0.5f);
                robot.backTankDrive.drive(-1000, 1000, 0.25f);
                robot.backTankDrive.drive(-3000, -3000, 0.5f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(1000, 1000, 0.5f);
                break;
            case BLUE:
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(-3120, -3120, 0.5f);
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(straight, -straight, 0.5f);
                robot.backTankDrive.drive(1000, -1000, 0.25f);
                robot.backTankDrive.drive(-3000, -3000, 0.5f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(1000, 1000, 0.5f);
                break;

            default:

                break;
        }

    }


}