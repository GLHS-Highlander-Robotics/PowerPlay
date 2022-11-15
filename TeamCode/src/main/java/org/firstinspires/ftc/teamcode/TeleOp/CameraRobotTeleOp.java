package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.OldRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.FullDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Camera Robot TeleOp")
public class CameraRobotTeleOp extends LinearOpMode {

    FullDetection fullDetection = new FullDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        fullDetection = new FullDetection();
        camera.setPipeline(fullDetection);

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
            telemetry.addData("Is there a pole?: ", fullDetection.getPole());
            telemetry.addData("Percent Yellow: ", fullDetection.getPercent());
            telemetry.update();
        }
        OldRobot robot = new OldRobot(this, hardwareMap, telemetry, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gamepadPlusCamera(robot, fullDetection);
            robot.singleJointGripperArm.respondToGamepad();
            telemetry.addData("Is there a pole?: ", fullDetection.getPole());
            telemetry.addData("Percent Yellow: ", fullDetection.getPercent());
            telemetry.update();
        }
    }

    private void gamepadPlusCamera(OldRobot robot, FullDetection fullDetection) {
        robot.backTankDrive.respondToGamepad();
        if (robot.backTankDrive.gamepad.right_bumper) {
            if (fullDetection.getPole()) {
                robot.backTankDrive.backLeftMotor.setPower(0);
                robot.backTankDrive.backRightMotor.setPower(0);
            } else {
                robot.backTankDrive.backLeftMotor.setPower(-0.25);
                robot.backTankDrive.backRightMotor.setPower(0.25);
            }
        }
        if (robot.backTankDrive.gamepad.left_bumper) {
            if (fullDetection.getPole()) {
                robot.backTankDrive.backLeftMotor.setPower(0);
                robot.backTankDrive.backRightMotor.setPower(0);
            } else {
                robot.backTankDrive.backLeftMotor.setPower(0.25);
                robot.backTankDrive.backRightMotor.setPower(-0.25);
                //abcdefghijklmnopqrstuvwxyz
                //The quick brown fox jumped over the lazy dog
                //314159265358979326538
            }
        }
    }
}
