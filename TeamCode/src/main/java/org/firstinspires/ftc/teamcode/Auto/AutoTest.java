package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetectionRight;
import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetectionRight.ParkingPosition;

@Autonomous(name = "Autonomous Test")
public class AutoTest extends RobotOpMode{
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, -20, 1125, 0.45, 1);
    private final SleeveDetectionRight sleeve = new SleeveDetectionRight();
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

        drive.strafeInches(20, 0.5f);
        drive.driveInches(56, 48,0.5f);
        drive.strafeInches(-8, 0.2f);
        slide.setSlide(4150, true);
        drive.driveInches(2,2, 0.2f);
        slide.ungrab();
        drive.driveInches(-2,-2,0.2f);
        slide.setSlide(300, true);
//        position = sleeve.getPosition();
//        drive.driveInches(-2250,-2250, 0.5f);
        drive.drive(-850,850,0.25f);
//        cycle(700);
    }
    public void cycle(int height){
        drive.drive(-850,850,0.25f);
        drive.driveInches(33,33,0.45f);
        slide.setSlide(height,true);
        drive.driveInches(4,4,0.25f);
        slide.grab();
        drive.driveInches(-1,-1,0.15f);
        slide.setSlide(height+600,true);
//        drive.driveInches(-2, -2,0.25f);
//        slide.setSlide(900,true);
        drive.driveInches(-13,-10,0.25f);
        slide.setSlide(300,true);
        drive.driveInches(-25,-25,0.45f);
        drive.drive(875,-875, 0.25f);
        slide.setSlide(4150, true);
        drive.driveInches(2,2,0.15f);
        slide.ungrab();
        drive.driveInches(-2,-2,0.15f);
        slide.setSlide(0,true);
    }
}
