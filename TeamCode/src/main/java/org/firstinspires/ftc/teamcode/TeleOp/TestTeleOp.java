package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;

@TeleOp(name = "Tele Op Test")
public class TestTeleOp extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, 0, 450, 0.45, 1);

    @Override
    public void main() {
        waitForStart();

        while (opModeIsActive()) {
            drive.updateByGamepad();
            slide.updateByGamepad();
        }
    }
}
