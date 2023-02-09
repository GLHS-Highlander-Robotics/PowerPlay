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

        drive.imu.resetYaw();
        //You need to have wait for start or else bad things happen
        waitForStart();
        while (colorsensor.getHue() > 80) {
            colorsensor.update();
            drive.setPowers(-0.2f);
            telemetry.addData("Hue", colorsensor.getHue());
            telemetry.update();
        }
        drive.setPowers(0);
//        drive.driveInches(12,12,0.3f);
//        drive.turnToHeading(0.5, 90);


    }
}
