package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StrafeDrive;
import org.firstinspires.ftc.teamcode.slide.LinearSlide;

@Autonomous(name = "Auto Test")
@Disabled
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        StrafeDrive drive = new StrafeDrive(hardwareMap);
        LinearSlide slide = new LinearSlide(hardwareMap);

        slide.release();

        waitForStart();

        if (isStopRequested()) return;

        // First Cone
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                //.addDisplacementMarker(slide::grab)
                .forward(5.5)
                .strafeLeft(37.5)
                //.addDisplacementMarker(() -> slide.setSlideAndWait(1900))
                .forward(6.5)
                //.addDisplacementMarker(slide::release)
                .build();

        drive.followTrajectory(trajectory);
    }
}
