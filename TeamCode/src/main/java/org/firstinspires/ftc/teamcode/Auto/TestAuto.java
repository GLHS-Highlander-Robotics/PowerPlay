package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;

@Autonomous(name = "Autonomous Test")
public class TestAuto extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, -20, 1125, 0.45, 1);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide);

        // Put motors in encoder mode
        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.ungrab();
        //You need to have wait for start or else bad things happen
        waitForStart();
        slide.grab();
        drive.driveInches(5.5, 5.5, 0.2f);
        slide.setSlide(200, true);
        drive.strafeInches(37.5, 0.5f);
        slide.setSlide(475, true);
        drive.driveInches(7, 7, 0.2f);
        slide.ungrab();
        drive.strafeInches(14.0, 0.5f);
        slide.setSlide(155, true);
        drive.driveInches(20.5, 20.5, 0.5f);
        slide.grab();
        slide.setSlide(250, true);
        drive.driveInches(-1, -1, 0.25f);
        slide.setSlide(400, true);
        drive.driveInches(-5, -5, 0.25f);
        slide.setSlide(20, true);
        drive.driveInches(-42, -42, 0.5f);
        drive.strafeInches(-10.0, 0.5f);
        slide.setSlide(750, true);
        drive.driveInches(7, 7, 0.2f);
        slide.ungrab();
        drive.driveInches(-7, -7, 0.2f);
        drive.strafeInches(16.0, 0.5f);
        drive.driveInches(42, 42, 0.5f);
        slide.setSlide(200, true);
        slide.grab();

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
