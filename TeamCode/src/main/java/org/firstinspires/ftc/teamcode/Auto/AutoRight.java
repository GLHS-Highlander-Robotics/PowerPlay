package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetectionRight;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous(name = "Autonomous Right")
public class AutoRight extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, -20, 1125, 0.45, 1);
    private final SleeveDetectionRight sleeve = new SleeveDetectionRight();
    private final Webcam cam = new Webcam(this, "Webcam 1", sleeve);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide, cam);

        // Put motors in encoder mode
        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.ungrab();
        SleeveDetectionRight.ParkingPosition position = sleeve.getPosition();
        //You need to have wait for start or else bad things happen
        waitForStart();
        position = sleeve.getPosition();
        //Place First Cone
        slide.grab();
        drive.driveInches(5.5, 5.5, 0.2f);
        slide.setSlide(200, true);
        drive.strafeInches(-37.5, 0.5f);
        slide.setSlide(475, true);
        drive.driveInches(6.5, 6.5, 0.2f);
        slide.ungrab();

        //Go for Second Cone
        drive.strafeInches(-14.0, 0.5f);
        slide.setSlide(125, true);
        drive.driveInches(20, 20, 0.5f);
        slide.grab();
        slide.setSlide(250, true);
        drive.driveInches(-1, -1, 0.25f);
        slide.setSlide(375, true);
        drive.driveInches(-5, -5, 0.25f);
        slide.setSlide(75, true);
        drive.driveInches(-44, -44, 0.5f);
        drive.strafeInches(12.5, 0.5f);
        slide.setSlide(800, true);
        drive.driveInches(4, 4, 0.2f);
        slide.ungrab();
        drive.driveInches(-4, -4, 0.2f);
        slide.setSlide(70, true);
        drive.driveInches(0, 1.5, 0.5f);
        drive.strafeInches(-12, 0.5f);
        drive.driveInches(50, 50, 0.3f);

        //Third Cone
        slide.grab();
        slide.setSlide(150, true);
        drive.driveInches(-1, -1, 0.25f);
        slide.setSlide(375, true);
        drive.driveInches(-5, -5, 0.25f);
        slide.setSlide(50, true);
        drive.driveInches(-38, -38, 0.5f);
        slide.setSlide(1000, true);
        drive.drive(700, -700, 0.2f);
        drive.driveInches(4, 4, 0.2f);
        slide.ungrab();
        drive.driveInches(-4, -4, 0.2f);
        drive.drive(-700, 700, 0.2f);

        //Move to Parking
        switch (position) {
            case CENTER:
                drive.driveInches(10, 10, 0.2f);
                break;
            case RIGHT:
                drive.driveInches(-10, -10, 0.2f);
                break;
            case LEFT:
                drive.driveInches(20, 20, 0.2f);

        }
        /*
        1350 is Approximately 25 inches moving forwards
        1350 is 24 inches moving sideways
        270 rotation is approximately 35 degrees
        250 linear slide is 1.5 inches

        Directions
        strafe 39.5 inches
        place cone
        strafe 12 inches
        rotate 180 degrees
        move forwards 27 inches
        */

        //        robot.linearSlide.grab();
//        drive.driveInches(3, 3, 0.25f);
//        drive.strafeInches(39.5, 0.25f);

//        drive.drive(200, 200, 0.25f);

        //place first cone (low)

//        robot.linearSlide.unGrab();

//        drive.drive(-200, -200, 0.25f);zs
//        slide.setSlide(0);
//        drive.strafeInches(20, 0.25f);
//        robot.strafeDrive.drive(-926, 926, 0.5f);
//        drive.driveInches(27, 27, 0.25f);
        //grab second cone
//        slide.setSlide(300);

//        robot.linearSlide.grab();

//        slide.setSlide(450);
//        drive.driveInches(-3, -3, 0.25f);
//        slide.setSlide(50);

//        drive.driveInches(-37, -37, 0.25f);
//        drive.strafeInches(-10, 0.5f);
//        drive.drive(200, 200, 0.5f);
        //place second cone (mid)

//        slide.setSlide(300);
//        drive.drive(-200, -200, 0.5f);
//        drive.strafeInches(10, 0.5f);
//        drive.driveInches(40, 40, 0.5f);

        //grab third cone

//        drive.driveInches(-35, -35, 0.5f);
//        drive.drive(-463, 463, 0.5f);

//        place third cone

//        drive.drive(463, -463, 0.5f);
        //park

//        robot.strafeDrive.drive(1680, 1680, 0.5f);
//        robot.strafeDrive.strafe(1000, 0.5f);
//        robot.strafeDrive.strafe(-1000, 0.5f);
//        robot.strafeDrive.drive(1000, -1000, 0.5f);
//        robot.strafeDrive.drive(-520,-520, 0.5f);
//        robot.strafeDrive.drive(-520,-520, -0.5f);

    }
}
