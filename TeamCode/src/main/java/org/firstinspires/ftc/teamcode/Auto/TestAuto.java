package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;

@Autonomous(name = "Autonomous Test")
public class TestAuto extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this, 0, 450, 0.45, 1);

    @Override
    public void main() {
        // Put motors in encoder mode
        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //You need to have wait for start or else bad things happen
        waitForStart();

        /*
        1350 is Approximately 25 inches moving forwards
        1350 is 24 inches moving sideways
        270 rotation is approximately 35 degrees

        Directions
        strafe 39.5 inches
        place cone
        strafe 12 inches
        rotate 180 degrees
        move forwards 27 inches
        */

        //        robot.linearSlide.grab();
        drive.strafeInches(39.5, 0.5f);
//        robot.linearSlide.setSlide();
        drive.drive(200, 200, 0.5f);
        //place first cone (low)
        slide.setSlide(150);
//        robot.linearSlide.ungrab();
        slide.setSlide(0);
        drive.drive(-200, -200, 0.5f);
        drive.strafeInches(12, 0.5f);
//        robot.strafeDrive.drive(-926, 926, 0.5f);
        drive.driveInches(27, 27, 0.5f);
        //grab second cone
        slide.setSlide(300);
//        robot.linearSlide.grab();
        slide.setSlide(450);
        drive.driveInches(-3, -3, 0.5f);
        slide.setSlide(50);
        drive.driveInches(-37, -37, 0.5f);
        drive.strafeInches(-10, 0.5f);
        drive.drive(200, 200, 0.5f);
        //place second cone (mid)
        slide.setSlide(300);
        drive.drive(-200, -200, 0.5f);
        drive.strafeInches(10, 0.5f);
        drive.driveInches(40, 40, 0.5f);
        //grab third cone
        drive.driveInches(-35, -35, 0.5f);
        drive.drive(-463, 463, 0.5f);
        //place third cone
        drive.drive(463, -463, 0.5f);
        //park

//        robot.strafeDrive.drive(1680, 1680, 0.5f);
//        robot.strafeDrive.strafe(1000, 0.5f);
//        robot.strafeDrive.strafe(-1000, 0.5f);
//        robot.strafeDrive.drive(1000, -1000, 0.5f);
//        robot.strafeDrive.drive(-520,-520, 0.5f);
//        robot.strafeDrive.drive(-520,-520, -0.5f);

    }
}
