package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.pipeline.SleeveDetectionNewOld;
import org.firstinspires.ftc.teamcode.subsystem.colorsensor.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystem.webcam.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class AutoFunny extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LinearSlide slide = new LinearSlide(hardwareMap);
        SleeveDetectionNewOld sleeveDetection = new SleeveDetectionNewOld();
        Webcam cam = new Webcam(hardwareMap, sleeveDetection);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);

//        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-36.85, -66.01, Math.toRadians(88.15)))
//                .splineTo(new Vector2d(-6.40, -15.31), Math.toRadians(2.23))
//                .splineToConstantHeading(new Vector2d(26.97, -56.94), Math.toRadians(199.98))
//                .splineToConstantHeading(new Vector2d(-37.01, -12.88), Math.toRadians(175.60))
//                .build();
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-32.48, -67.46, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-13.20, -52.56), Math.toRadians(88.52))
                .splineToConstantHeading(new Vector2d(-20.65, -4.13), Math.toRadians(128.21))
                .build();
        drive.setPoseEstimate(traj.start());

        drive.setPoseEstimate(traj.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj);
    }
}
