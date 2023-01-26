package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetectionLeft;
import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetectionLeft.ParkingPosition;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous(name = "Autonomous Test")
public class AutoTest extends RobotOpMode{
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, -20, 1125, 0.45, 1);
    private final SleeveDetectionLeft sleeve = new SleeveDetectionLeft(20, 250);
    private final Webcam cam = new Webcam(this, "Webcam 1", sleeve);
    @Override
    public void runOpMode(){
        addSubsystems(drive, slide, cam);

        // Put motors in encoder mode
        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.ungrab();
        ParkingPosition position = sleeve.getPosition();
        //You need to have wait for start or else bad things happen
        waitForStart();
//        position = sleeve.getPosition();
//        drive.driveInches(-2250,-2250, 0.5f);
        cycle(640);
    }
    public void cycle(int height){
        drive.drive(-875,875,0.25f);
        drive.driveInches(36,36,0.25f);
        slide.setSlide(height,true);
        slide.grab();
        slide.setSlide(height+25,true);
        drive.driveInches(-2, -2,0.25f);
        slide.setSlide(325,true);
        drive.driveInches(-8,-8,0.25f);
        slide.setSlide(50,true);
        drive.driveInches(-28,-28,0.25f);
        drive.drive(875,-875, 0.25f);
        slide.setSlide(1000, true);
        drive.driveInches(4,4,0.25f);
        slide.ungrab();
        drive.driveInches(-4,-4,0.25f);
        slide.setSlide(0,true);
    }
}
