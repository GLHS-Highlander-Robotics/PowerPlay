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
    public void runOpMode() {
        addSubsystems(drive, slide);

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

        drive.strafeInches(-39.5, 0.5f);
        drive.strafe(200, 0.5f);

        drive.strafeInches(-12, 0.5f);
        drive.drive(-926, 926, 0.5f);
        drive.strafeInches(-27, 0.5f);
    }
}
