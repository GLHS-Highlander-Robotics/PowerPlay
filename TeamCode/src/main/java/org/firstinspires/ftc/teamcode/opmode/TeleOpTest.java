package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StrafeDrive;

@TeleOp(name = "TeleOp Test")
public class TeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        StrafeDrive drive = new StrafeDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        double limiter = 0.75;
        boolean field = true;

        while (opModeIsActive()) {
            double heading = drive.getExternalHeading();

            if (gamepad1.a) {
                limiter = 0.75;
            } else if (gamepad1.b) {
                limiter = 0.25;
            }
            if (gamepad1.x) {
                drive.setPoseEstimate(new Pose2d());
                field = true;
            } else if (gamepad1.y) {
                field = false;
            }

            double forward = -gamepad1.left_stick_y * limiter;
            double strafe = gamepad1.left_stick_x * limiter;
            double rotate = gamepad1.right_stick_x * limiter;

            if (Math.abs(-gamepad1.left_stick_y) < 0.01) {
                forward = -gamepad2.left_stick_y * 0.4;
                if (Math.abs(gamepad2.left_stick_y) < 0.02) {
                    forward = 0;
                }
            }
            if (Math.abs(gamepad1.left_stick_x) < 0.01) {
                strafe = gamepad2.left_stick_x * 0.4;
                if (Math.abs(gamepad2.left_stick_x) < 0.02) {
                    strafe = 0;
                }
            }
            if (Math.abs(gamepad1.right_stick_x) < 0.01) {
                rotate = gamepad2.right_stick_x * 0.4;
                if (Math.abs(gamepad2.right_stick_x) < 0.02) {
                    rotate = 0;
                }
            }
            double fieldForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            double fieldStrafe = strafe * Math.cos(-heading) - forward * Math.sin(-heading);

            if (field) {
                drive.setMotorPowers(
                        fieldForward + fieldStrafe + rotate,
                        fieldForward - fieldStrafe + rotate,
                        fieldForward - fieldStrafe - rotate,
                        fieldForward + fieldStrafe - rotate
                );
            } else {
                drive.setMotorPowers(
                        forward + strafe + rotate,
                        forward - strafe + rotate,
                        forward - strafe - rotate,
                        forward + strafe - rotate
                );
            }

            telemetry.addData("Limiter: ", limiter);
            telemetry.addData("Heading: ", heading);
            telemetry.addData("Field Centric?: ", field);

            drive.update();
        }
    }
}
