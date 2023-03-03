package org.firstinspires.ftc.teamcode.old.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.old.RobotOpMode;
import org.firstinspires.ftc.teamcode.old.pipeline.SleeveDetectionRightOld;
import org.firstinspires.ftc.teamcode.old.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.old.subsystem.StrafeDrive;
import org.firstinspires.ftc.teamcode.old.subsystem.Webcam;

@Disabled
@Autonomous(name = "Jumpscare")
public class Jumpscare extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);
    private final SleeveDetectionRightOld sleeve = new SleeveDetectionRightOld();
    private final Webcam cam = new Webcam(this, "Webcam 1", sleeve);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide, cam);
        waitForStart();
        drive.strafeInches(5, 0.5f);
        drive.driveInches(-6, -6, 0.5f);
    }
}
