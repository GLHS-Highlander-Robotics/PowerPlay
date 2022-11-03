package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.Util.Maths;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Signal Sleeve Test")
public class VisionTest extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    private DcMotor arm;
    private Servo grip1;
    private Servo grip2;
    private int lPos;
    private int rPos;
    private double gripMin = 0.45;
    private double gripMax = 1.0;

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
                robot.backTankDrive.drive(-520, 520, 0.5f);
                robot.backTankDrive.drive(-400, -400, 0.25f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(520, 520, 0.25f);
                robot.singleJointGripperArm.setArm(0);
                robot.backTankDrive.drive(520, -520, 0.5f);
                robot.backTankDrive.drive(3000, 3000, 0.5f);
                robot.backTankDrive.drive(500, -500, 0.5f);
                robot.backTankDrive.drive(-3000, -3000, 0.5f);
                robot.backTankDrive.drive(-500, 500, 0.5f);
                robot.backTankDrive.drive(-2000, -2000, 0.5f);
                break;
            case RIGHT:
                robot.backTankDrive.drive(-2080, -2080, 0.50f);
                robot.backTankDrive.drive(-520, 520, 0.50f);
                robot.backTankDrive.drive(-2080, -2080, 0.50f);
                robot.backTankDrive.drive(520, -520, 0.50f);
                break;
            default:
                robot.singleJointGripperArm.ungrab();
                robot.backTankDrive.drive(-3120, -3120, 0.5f);
                robot.singleJointGripperArm.setArm(450);
                robot.backTankDrive.drive(-520, 520, 0.5f);
                robot.backTankDrive.drive(-400, -400, 0.25f);
                robot.singleJointGripperArm.grab();
                robot.backTankDrive.drive(520, 520, 0.25f);
                robot.singleJointGripperArm.setArm(0);
                robot.backTankDrive.drive(375, -375, 0.5f);
                robot.backTankDrive.drive(-400, -400, 0.5f);
                break;
        }

    }

    private void move(int leftTarget, int rightTarget, double speed) {
        lPos += leftTarget;
        rPos += rightTarget;

        left.setTargetPosition(lPos);
        right.setTargetPosition(rPos);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(speed);

        while(opModeIsActive() && left.isBusy() && right.isBusy()) {
            idle();
        }
    }

    private void armSet(int steps) {
        arm.setTargetPosition(Maths.clamp(steps, 0, 450));
        while (opModeIsActive() && arm.isBusy()) {
            idle();
        }
    }
    public void grab() {
        grip1.setPosition(1-gripMax);
        grip2.setPosition(gripMax);
    }

    public void ungrab() {
        grip1.setPosition(1-gripMin);
        grip2.setPosition(gripMin);
    }
}