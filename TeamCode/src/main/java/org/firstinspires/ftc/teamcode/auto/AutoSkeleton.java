package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.ACTUATOR_MAX;
import static org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide.ACTUATOR_MIN;

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
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystem.webcam.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class AutoSkeleton extends LinearOpMode {
    public static double x = -27;
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

        int[] heights = {480, 360, 180, 0, 0};
        int time = 0;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(slide::grab)
                .addDisplacementMarker(() -> slide.setSlide(60))
                .splineToLinearHeading(new Pose2d(10, -22, Math.toRadians(55)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(58, -21, Math.toRadians(55)), Math.toRadians(0))
                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MAX_HEIGHT))
                .forward(4.5)
                .waitSeconds(1.5)
                .addDisplacementMarker(slide::ungrab)
                .waitSeconds(1.5)
                .back(4.5)
                .addDisplacementMarker(() -> slide.setSlide(60))
                .splineToLinearHeading(new Pose2d(50, -10, Math.toRadians(0)), Math.toRadians(0))
                .build();


        TrajectorySequence trajRepeat = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(() -> slide.setSlide(480))
                .splineToLinearHeading(new Pose2d(0, 40, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(slide::grab)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MAX_HEIGHT))
                .forward(5)
                .waitSeconds(1.5)
                .addDisplacementMarker(slide::ungrab)
                .waitSeconds(1.5)
                .back(5)
                .build();

        TrajectorySequence trajA = drive.trajectorySequenceBuilder(new Pose2d(-31.67, -64.77, Math.toRadians(90.00)))
                .addDisplacementMarker(slide::grab)
                .splineToConstantHeading(new Vector2d(-12.08, -55.64), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-12.27, -12.46), Math.toRadians(90.00))
                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MAX_HEIGHT))
                .splineToConstantHeading(new Vector2d(-29.01, -9), Math.toRadians(180.00))
                .waitSeconds(1)
                .build();
        drive.setPoseEstimate(trajA.start());
//new Pose2d(-24.38, -7, Math.toRadians(90.00))
        TrajectorySequence trajB = drive.trajectorySequenceBuilder(trajA.end())

                .addDisplacementMarker(slide::ungrab)

                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MIN_HEIGHT))

//                .lineToConstantHeading(new Vector2d(-24.38, -10.00))
                //.splineToConstantHeading(new Vector2d(-45.61, -10.72), Math.toRadians(180.00))
                .addDisplacementMarker(() -> slide.setSlide(500))
                .lineToSplineHeading(new Pose2d(-66.5, -12, Math.toRadians(180.00)))
                .build();
//new Pose2d(-67.00, -7.78, Math.toRadians(180.00))
        TrajectorySequence trajC = drive.trajectorySequenceBuilder(trajB.end())
                .addDisplacementMarker(() -> {
                    slide.grab();
                    sleep(500);
                })
                .addDisplacementMarker(() -> {
                    slide.setSlide(600);
                    slide.setLinearActuator(ACTUATOR_MIN);

                    sleep(1000);
                })
//                .addDisplacementMarker(() -> slide.grabAndRaise(800))
                .lineToSplineHeading(new Pose2d(-27, -9, Math.toRadians(90.00)), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    slide.setSlide(LinearSlide.MAX_HEIGHT);
                    slide.setLinearActuator(ACTUATOR_MAX);
                    sleep(2000);
                })
                .lineToConstantHeading(new Vector2d(-27, -7), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    slide.ungrab();
                    sleep(500);
                })
                .build();
        TrajectorySequence trajD = drive.trajectorySequenceBuilder(trajC.end())
                .addDisplacementMarker(slide::ungrab)
                .addDisplacementMarker(() -> slide.setSlide(LinearSlide.MIN_HEIGHT))
//                .lineToConstantHeading(new Vector2d(-24.38, -10.00))
                .splineToConstantHeading(new Vector2d(-45.61, -10.72), Math.toRadians(180.00))
                .addDisplacementMarker(() -> slide.setSlide(360))
                .lineToSplineHeading(new Pose2d(-64.00, -7.78, Math.toRadians(180.00)))
                .build();
//new Pose2d(-67.00, -7.78, Math.toRadians(180.00))
        TrajectorySequence trajE = drive.trajectorySequenceBuilder(trajD.end())
                .addDisplacementMarker(() -> {
                    slide.grab();
                    sleep(500);
                })
                .addDisplacementMarker(() -> {
                    slide.setSlide(1200);
                    sleep(1000);
                })
//                .addDisplacementMarker(() -> slide.grabAndRaise(800))
                .lineToConstantHeading(new Vector2d(-48.61, -10.72))
                .lineToSplineHeading(new Pose2d(-24, -10.75, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    slide.setSlide(LinearSlide.MAX_HEIGHT);
                    sleep(1000);
                })
                .lineToConstantHeading(new Vector2d(-27, -3))
                .addDisplacementMarker(() -> {
                    slide.ungrab();
                    sleep(200);
                })
                .build();


        //Cone 1: 480
        //Cone 2: 360
        //Cone 3: 180
        //Cone 4: 0
        //Cone 5: 0
        slide.ungrab();
        drive.imu.resetYaw();
        slide.setLinearActuator(ACTUATOR_MAX);
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajA);

        drive.followTrajectorySequence(trajB);
        drive.followTrajectorySequence(trajC);
        drive.followTrajectorySequence(trajD);
        drive.followTrajectorySequence(trajE);
    }
}
