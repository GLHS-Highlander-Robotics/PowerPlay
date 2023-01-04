package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pipelines.PoleDetection;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.RearTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@TeleOp(name = "Camera Robot TeleOp")
public class CameraRobotTeleOp extends RobotOpMode {
    private final Arm arm = new Arm(this);
    private final PoleDetection poleDetection = new PoleDetection();
    private final Webcam camera = new Webcam(this, "Webcam 1", poleDetection);
    private final RearTankDrive drive = new RearTankDrive(this, 1) {
        @Override
        public void updateByGamepad() {
            super.updateByGamepad();
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                if (poleDetection.getPole()) {
                    setPowers(0);
                } else {
                    double leftPower = 0;
                    double rightPower = 0;
                    if (gamepad1.right_bumper) {
                        leftPower = -0.25;
                        rightPower = 0.25;
                    } else if (gamepad1.left_bumper) {
                        leftPower = 0.25;
                        rightPower = -0.25;
                    }
                    leftMotor.setPower(leftPower);
                    rightMotor.setPower(rightPower);
                }
            }
        }
    };

    @Override
    public void runOpMode() {
        addSubsystems(drive, arm, camera);

        while (!isStarted()) {
            telemetry.addData("Is there a pole?: ", poleDetection.getPole());
            telemetry.addData("Percent Yellow: ", poleDetection.getPercent());
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Is there a pole?: ", poleDetection.getPole());
            telemetry.addData("Percent Yellow: ", poleDetection.getPercent());
            telemetry.update();
            arm.updateByGamepad();
            drive.updateByGamepad();
        }
    }
}
