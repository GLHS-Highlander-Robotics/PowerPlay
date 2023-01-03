package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;

@TeleOp(name = "Test TeleOp")
public class TestTeleOp extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, 0, 450, 0.45, 1);

    public void setup() {
        addSubsystems(drive, slide);
    }

    public void onStart() {
        telemetry.addLine("Starting up");
    }

    public void update() {
        updateDrive();
    }

    public void updateDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (Math.abs(-gamepad1.left_stick_y) < 0.01) forward = 0;
        if (Math.abs(gamepad1.left_stick_x) < 0.01) strafe = 0;
        if (Math.abs(gamepad1.right_stick_x) < 0.01) rotate = 0;

        drive.frontLeftMotor.setPower(forward + strafe + rotate);
        drive.backLeftMotor.setPower(forward - strafe + rotate);
        drive.frontRightMotor.setPower(forward + strafe - rotate);
        drive.backRightMotor.setPower(forward - strafe - rotate);
    }
}
