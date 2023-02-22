package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystem.StrafeDrive;

@TeleOp(name = "Two Player TeleOp")
public class TwoPlayerTeleop extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide);
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
