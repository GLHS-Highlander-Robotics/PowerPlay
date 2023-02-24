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
public class AutoFunny extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LinearSlide slide = new LinearSlide(hardwareMap);
        SleeveDetectionNew sleeveDetection = new SleeveDetectionNew();
        Webcam cam = new Webcam(hardwareMap, sleeveDetection);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-36.48, -70.71, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-28.13, -13.82), Math.toRadians(33.69))
                .splineTo(new Vector2d(-19.93, -69.75), Math.toRadians(-80.09))
                .build();
        drive.setPoseEstimate(traj.start());

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj);
    }
}
