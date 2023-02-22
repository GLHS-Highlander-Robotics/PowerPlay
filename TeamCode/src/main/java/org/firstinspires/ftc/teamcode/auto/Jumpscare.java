package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.pipeline.SleeveDetectionRight;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystem.StrafeDrive;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

@Autonomous(name = "Jumpscare")
public class Jumpscare extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);
    private final SleeveDetectionRight sleeve = new SleeveDetectionRight();
    private final Webcam cam = new Webcam(this, "Webcam 1", sleeve);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide, cam);
        waitForStart();
        drive.strafeInches(5, 0.5f);
        drive.driveInches(-6, -6, 0.5f);
    }
}
