package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipeline.SleeveDetectionNew;
import org.firstinspires.ftc.teamcode.subsystem.colorsensor.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystem.webcam.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class AutoSkeleton extends LinearOpMode {
    public static double x = 58;
    public static double y = -21;
    public static double f = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LinearSlide slide = new LinearSlide(hardwareMap);
        SleeveDetectionNew sleeveDetection = new SleeveDetectionNew();
        Webcam cam = new Webcam(hardwareMap, sleeveDetection);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);

        int height = 500;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(slide::grab)
                .addDisplacementMarker(() -> slide.setSlide(60))
                .splineToLinearHeading(new Pose2d(10, -22, Math.toRadians(55)), Math.toRadians(0))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(x, y))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MAX_HEIGHT))
                .forward(4.5)
                .waitSeconds(1.5)
                .addDisplacementMarker(slide::ungrab)
                .waitSeconds(1.5)
                .back(4.5)
//                .addDisplacementMarker(() -> slide.setSlide(60))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .addDisplacementMarker(() -> slide.setSlide(60))
                .splineToLinearHeading(new Pose2d(50, -10, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence trajRepeat = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(() -> slide.setSlide(height))
                .splineToLinearHeading(new Pose2d(0, 40, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(slide::grab)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MAX_HEIGHT))
                .forward(4.5)
                .waitSeconds(1.5)
                .addDisplacementMarker(slide::ungrab)
                .waitSeconds(1.5)
                .back(4.5)
                .build();

        slide.ungrab();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);
        drive.followTrajectorySequence(traj4);
    }
}
