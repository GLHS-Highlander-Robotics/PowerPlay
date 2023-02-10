package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOp Test")
public class TeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        double limiter = 0.75;

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * limiter;
            double strafe = gamepad1.left_stick_x * limiter;
            double rotate = gamepad1.right_stick_x * limiter;
            double heading = drive.getExternalHeading();

            if (Math.abs(gamepad1.left_stick_y) < 0.01) {
                forward = 0;
            }
            if (Math.abs(gamepad1.left_stick_x) < 0.01) {
                strafe = 0;
            }
            if (Math.abs(gamepad1.right_stick_x) < 0.01) {
                rotate = 0;
            }

            double fieldForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            double fieldStrafe = strafe * Math.cos(-heading) - forward * Math.sin(-heading);

            drive.setMotorPowers(
                    fieldForward + fieldStrafe + rotate,
                    fieldForward - fieldStrafe + rotate,
                    fieldForward - fieldStrafe - rotate,
                    fieldForward + fieldStrafe - rotate
            );

            drive.update();
        }
    }
}
