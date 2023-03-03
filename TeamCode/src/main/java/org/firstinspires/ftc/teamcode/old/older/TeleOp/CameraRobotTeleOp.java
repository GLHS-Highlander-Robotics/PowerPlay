package org.firstinspires.ftc.teamcode.old.older.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.old.older.Robots.OldRobot;
import org.firstinspires.ftc.teamcode.old.Subsystems.PoleDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
public class CameraRobotTeleOp extends LinearOpMode {

    PoleDetection poleDetection = new PoleDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        poleDetection = new PoleDetection();
        camera.setPipeline(poleDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("Is there a pole?: ", poleDetection.getPole());
            telemetry.addData("Percent Yellow: ", poleDetection.getPercent());
            telemetry.update();
        }
        OldRobot robot = new OldRobot(this, hardwareMap, telemetry, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gamepadPlusCamera(robot, poleDetection);
            robot.singleJointGripperArm.respondToGamepad();
            telemetry.addData("Is there a pole?: ", poleDetection.getPole());
            telemetry.addData("Percent Yellow: ", poleDetection.getPercent());
            telemetry.update();
        }
    }

    private void gamepadPlusCamera(OldRobot robot, PoleDetection poleDetection) {
        robot.backTankDrive.respondToGamepad();
        if (robot.backTankDrive.gamepad.right_bumper) {
            if (poleDetection.getPole()) {
                robot.backTankDrive.backLeftMotor.setPower(0);
                robot.backTankDrive.backRightMotor.setPower(0);
            } else {
                robot.backTankDrive.backLeftMotor.setPower(-0.25);
                robot.backTankDrive.backRightMotor.setPower(0.25);
            }
        }
        if (robot.backTankDrive.gamepad.left_bumper) {
            if (poleDetection.getPole()) {
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
