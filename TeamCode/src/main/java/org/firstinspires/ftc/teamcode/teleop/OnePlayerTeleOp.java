package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;

@Config
@TeleOp
public class OnePlayerTeleOp extends LinearOpMode {
    public static double HIGH_POWER = 0.85;
    public static double LOW_POWER = 0.35;
    public static double DEAD_ZONE_P1 = 0.05;
    public static double DEAD_ZONE_P2 = 0.05;

    SampleMecanumDrive drive;
    LinearSlide slide;

    double limiter = HIGH_POWER;
    boolean fieldCentric = false;
    double botHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new LinearSlide(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            updateDriveByGamepad();
            drive.update();

            telemetry.addData("Limiter: ", limiter);
            telemetry.addData("Heading: ", botHeading);
            telemetry.addData("Field Centric?: ", fieldCentric);
            telemetry.update();
        }
    }

    public void updateDriveByGamepad() {
        botHeading = drive.getRawExternalHeading();

        if (gamepad1.a) {
            limiter = HIGH_POWER;
        } else if (gamepad1.b) {
            limiter = LOW_POWER;
        }

        if (gamepad1.x) {
            drive.setPoseEstimate(new Pose2d());
            fieldCentric = true;
        } else if (gamepad1.y) {
            fieldCentric = false;
        }

        double forward = -gamepad1.left_stick_y * limiter;
        double strafe = gamepad1.left_stick_x * limiter;
        double rotate = gamepad1.right_stick_x * limiter;

        if (Math.abs(gamepad1.left_stick_y) < DEAD_ZONE_P1) {
            forward = -gamepad2.left_stick_y * LOW_POWER;
            if (Math.abs(gamepad2.left_stick_y) < DEAD_ZONE_P2) {
                forward = 0;
            }
        }

        if (Math.abs(gamepad1.left_stick_x) < DEAD_ZONE_P1) {
            strafe = gamepad2.left_stick_x * LOW_POWER;
            if (Math.abs(gamepad2.left_stick_x) < DEAD_ZONE_P2) {
                strafe = 0;
            }
        }

        if (Math.abs(gamepad1.right_stick_x) < DEAD_ZONE_P1) {
            rotate = gamepad2.right_stick_x * LOW_POWER;
            if (Math.abs(gamepad2.right_stick_x) < DEAD_ZONE_P2) {
                rotate = 0;
            }
        }

        double fieldForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double fieldStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);

        if (!fieldCentric) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            forward,
                            -strafe,
                            -rotate
                    )
            );
        } else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            fieldForward,
                            -fieldStrafe,
                            -rotate
                    )
            );
        }
    }
}