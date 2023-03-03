package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "tuning")
public class SplineTest extends LinearOpMode {
    public static int TRIALS = 1;
    public static int DELAY = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        for (int i = 0; i < TRIALS; i++) {
            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), 0)
                    .build();

            drive.followTrajectory(traj);

            sleep(DELAY);

            drive.followTrajectory(
                    drive.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );

            sleep(DELAY);
        }
    }
}