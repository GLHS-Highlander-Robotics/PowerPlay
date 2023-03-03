package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.ACTUATOR_MAX;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.ACTUATOR_MIN;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.INCREMENT_STEPS;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.LOW_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MEDIUM_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.MIN_HEIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;

@Config
@TeleOp
public class TwoPlayerTeleOp extends LinearOpMode {
    public static double HIGH_POWER = 0.85;
    public static double LOW_POWER = 0.35;
    public static double DEAD_ZONE_P1 = 0.05;
    public static double DEAD_ZONE_P2 = 0.05;

    SampleMecanumDrive drive;
    LinearSlide slide;

    double limiter = HIGH_POWER;
    boolean fieldCentric = false;
    double botHeading;

    int armMotorSteps = 0;
    boolean dPadPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        slide = new LinearSlide(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.imu.resetYaw();
        while (opModeInInit()) {
            telemetry.addData("Linear Actuator Position:", slide.linearActuator.getPosition());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            updateDriveByGamepad();
            updateSlideByGamepad();

            drive.update();
            slide.update();

            telemetry.addData("Limiter: ", limiter);
            telemetry.addData("Heading: ", botHeading);
            telemetry.addData("Field Centric?: ", fieldCentric);

            telemetry.addData("Target arm motor steps:", armMotorSteps);
            telemetry.addData("Actual arm motor steps:", slide.slideMotor.getCurrentPosition());
            telemetry.addData("Arm Power:", slide.slideMotor.getPower());
            telemetry.addData("Arm Current (A):", slide.slideMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Arm ZeroPower:", slide.slideMotor.getZeroPowerBehavior());
            telemetry.addData("Linear Actuator Position:", slide.linearActuator.getPosition());

            telemetry.update();
        }
    }

    public void updateSlideByGamepad() {
        // Set arm steps to predefined height with buttons
        if (gamepad2.a) {
            armMotorSteps = MIN_HEIGHT;
        } else if (gamepad2.b) {
            armMotorSteps = LOW_HEIGHT;
        } else if (gamepad2.x) {
            armMotorSteps = MEDIUM_HEIGHT;
        } else if (gamepad2.y) {
            armMotorSteps = MAX_HEIGHT;
        }

        // Increase arm steps by DPAD increments
        if (gamepad2.dpad_up) {
            armMotorSteps += INCREMENT_STEPS;
            dPadPressed = true;
        } else if (gamepad2.dpad_down) {
            armMotorSteps -= INCREMENT_STEPS;
            dPadPressed = true;
        } else if (dPadPressed) {
            dPadPressed = false;
        }

        slide.setSlide(armMotorSteps);

        if (gamepad1.right_trigger > 0.5) {
            slide.grab();
        } else if (gamepad1.left_trigger > 0.5) {
            slide.ungrab();
        }

        if (gamepad2.right_trigger > 0.5) {
            slide.setLinearActuator(ACTUATOR_MIN);

        } else if (gamepad2.left_trigger > 0.5) {
            slide.setLinearActuator(ACTUATOR_MAX);

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
            drive.imu.resetYaw();
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