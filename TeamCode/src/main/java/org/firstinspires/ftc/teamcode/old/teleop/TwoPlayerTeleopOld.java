package org.firstinspires.ftc.teamcode.old.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.old.RobotOpMode;
import org.firstinspires.ftc.teamcode.old.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.old.subsystem.StrafeDrive;

@TeleOp(name = "Two Player TeleOp Old")
public class TwoPlayerTeleopOld extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide);
        slide.setLinearActuator(LinearSlide.GRIP_MAX);
        waitForStart();

        while (opModeIsActive()) {
            drive.updateByTwoGamepads();
            drive.updateTelemetry();
            slide.readGamepad();
            slide.updateTelemetry();
            telemetry.update();
        }
    }
}
