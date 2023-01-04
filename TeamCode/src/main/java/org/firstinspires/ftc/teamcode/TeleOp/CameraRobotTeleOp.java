package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pipelines.PoleDetection;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.RearTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp(name = "Camera Robot TeleOp")
public class CameraRobotTeleOp extends RobotOpMode {
    private final RearTankDrive drive = new RearTankDrive(
            this,
            1,
            5000,
            0.1
    );
    private final Arm arm = new Arm(this);
    private final PoleDetection poleDetection = new PoleDetection();
    private final Webcam camera = new Webcam(this, "Webcam 1", poleDetection);

    @Override
    public void setup() {
        addSubsystems(drive, arm, camera);

        while (!isStarted()) {
            telemetry.addData("Is there a pole?: ", poleDetection.getPole());
            telemetry.addData("Percent Yellow: ", poleDetection.getPercent());
            telemetry.update();
        }
    }

    @Override
    public void onStart() {
    }

    @Override
    public void update() {
        telemetry.addData("Is there a pole?: ", poleDetection.getPole());
        telemetry.addData("Percent Yellow: ", poleDetection.getPercent());
        gamepadPlusCamera();
        arm.updateByGamepad();
    }

    @Override
    public void onStop() {
    }

    private void gamepadPlusCamera() {
        drive.updateByGamepad();
        if (gamepad1.right_bumper) {
            if (poleDetection.getPole()) {
                drive.setPowers(0);
            } else {
                drive.leftMotor.setPower(-0.25);
                drive.rightMotor.setPower(0.25);
            }
        }
        if (gamepad1.left_bumper) {
            if (poleDetection.getPole()) {
                drive.setPowers(0);
            } else {
                drive.leftMotor.setPower(0.25);
                drive.rightMotor.setPower(-0.25);
            }
        }
    }
}
