package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.pipeline.PoleDetection;
import org.firstinspires.ftc.teamcode.pipeline.SleeveDetectionNew;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystem.SensorC;
import org.firstinspires.ftc.teamcode.subsystem.StrafeDrive;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

@Autonomous(name = "Autonomous Gyro Test Left")
public class AutonomousUpdated extends RobotOpMode {
    private final StrafeDrive drive = new StrafeDrive(this);
    private final LinearSlide slide = new LinearSlide(this);
    private final SleeveDetectionNew sleeve = new SleeveDetectionNew();
    private final PoleDetection pole = new PoleDetection();
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
        slide.grabAndWait();
        slide.setSlide(200);
        drive.strafeInches(20, 0.5f);
        drive.rotateAndMove(4.5, 45, 0.3, 0, -0.3);


    }
}
