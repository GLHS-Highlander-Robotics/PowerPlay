package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;

@TeleOp(name = "Test TeleOp")
public class TestTeleOp extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, 0, 450, 0.45, 1);

    @Override
    public void setup() {
        addSubsystems(drive, slide);
    }

    @Override
    public void onStart() {
    }

    @Override
    public void update() {
        drive.moveByGamepad();
    }

    @Override
    public void onStop() {
    }
}
