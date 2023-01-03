package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test TeleOp")
public class TestTeleOp extends RobotOpMode {
    Drive drive = new Drive(this);
    LinearSlide slide = new LinearSlide(this, 0, 450, 0.45, 1);

    public void setup() {
        addSubsystems(drive, slide);
    }

    public void update() {
        // executive update code
    }
}
