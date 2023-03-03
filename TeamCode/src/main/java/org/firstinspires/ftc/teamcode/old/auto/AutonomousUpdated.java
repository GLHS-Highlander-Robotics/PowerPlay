package org.firstinspires.ftc.teamcode.old.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.old.RobotOpMode;
import org.firstinspires.ftc.teamcode.old.pipeline.PoleDetectionOld;
import org.firstinspires.ftc.teamcode.old.pipeline.SleeveDetectionNewOld;
import org.firstinspires.ftc.teamcode.old.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.old.subsystem.SensorC;
import org.firstinspires.ftc.teamcode.old.subsystem.StrafeDrive;
import org.firstinspires.ftc.teamcode.old.subsystem.Webcam;

@Disabled
@Autonomous(name = "Autonomous Gyro Test Left")
public class AutonomousUpdated extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);
    private final SleeveDetectionNewOld sleeve = new SleeveDetectionNewOld();
    private final PoleDetectionOld pole = new PoleDetectionOld();
    private final Webcam cam = new Webcam(this, "Webcam 1", sleeve);
    private final SensorC colorsensor = new SensorC(this);

    @Override
    public void runOpMode() {
        addSubsystems(drive, slide, cam, colorsensor);


        // Put motors in encoder mode

        slide.release();
        while (opModeInInit()) {
            drive.updateHeadingDeg();
            telemetry.addData("Heading: ", drive.botHeading);
            telemetry.update();
        }
        waitForStart();
        drive.imu.resetYaw();
//        slide.grabAndWait();
//        slide.setSlide(200);
//        drive.strafeInches(20, 0.5f);
//        drive.rotateAndMove(4.5, 45, 0.3, 0, -0.3);
        drive.rotateAndMoveInches(0, 2, 20, 0.5, 0);
        drive.rotateAndMoveInches(0, 57, 0, 0.5, 0);
        slide.setSlideAndWait(4000);
        drive.rotateAndMoveInches(0, 0, -12, 0.5, 0);
        doTheFunny(drive, slide, 490);
        doTheFunny(drive, slide, 425);
    }

    public static void doTheFunny(StrafeDrive drive, LinearSlide slide, int height) {
        toStack(drive);
        coneFromStack(drive, slide, height);
    }

    public static void toStack(StrafeDrive drive) {
        drive.rotateAndMove(3.05, 77, 0, -0.3, -0.3);
//        drive.rotateAndMoveInches(77,0,45,0.3,0.3);
//        drive.rotateAndMove(0.25, 75, 0.3, 0, 0);
    }

    public static void coneFromStack(StrafeDrive drive, LinearSlide slide, int height) {
        slide.setSlideAndWait(height);
        slide.grabAndWait();
        slide.setSlideAndWait(height + 800);
        drive.rotateAndMove(0.95, 0, 0, 0.3, 0);
        //drive.driveInches(-8, -8, 0.3f);
        slide.setSlide(400);
        drive.rotateAndMove(2.32, 7, 0, 0.3, 0.3);
        slide.setSlideAndWait(3870);
        drive.driveInches(6, 6, 0.15f);
//        drive.driveInches(-2, -2, 0.15f);
        slide.releaseAndWait();
//        drive.driveInches(-1, -1, 0.15f);
        slide.setSlide(200);
//        drive.rotateAndMove(2.5, 0, 0.3, 0, -0.3);
//        slide.grab();
//        slide.setSlide(height + 75);
//        drive.driveStraight(0.5f, -4, 0);
//        drive.rotateAndMove(4.5,90, 0.3, 0, -0.3);
    }

    public void lineUpLarry(double color, float speed) {
        drive.setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        while (colorsensor.getHue() < (color - 10) || colorsensor.getHue() > (color + 10)) {
            drive.driveBot(0, 0.3, 0);
        }
        drive.setPowers(0);
    }
}
