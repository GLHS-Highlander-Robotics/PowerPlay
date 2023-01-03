package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.RearTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;
import org.firstinspires.ftc.teamcode.old.Subsystems.BoeDetection;

@Autonomous(name = "Test Auto")
public class TestAuto extends RobotOpMode {
    private final RearTankDrive drive = new RearTankDrive(this);
    private final BoeDetection coneDetection = new BoeDetection();
    private final Webcam camera = new Webcam(this, "Webcam 1", coneDetection);
    private final Arm arm = new Arm(this);

    @Override
    public void setup() {
        addSubsystems(drive, camera, arm);

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", coneDetection.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void onStart() {
        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.grab();

        int straight = 0;
        boolean turningRight = true;
        while (coneDetection.getPosition() == BoeDetection.Cone.UNDECIDED || coneDetection.getPosition() == BoeDetection.Cone.RED) {
            if (turningRight) {
                drive.drive(-10, 10, 0.5f);
                straight += 10;
            } else {
                drive.drive(10, -10, 0.5f);
                straight -= 10;
            }
            if (straight >= 500) {
                turningRight = false;
            } else if (straight <= -500) {
                turningRight = true;
            }
        }

        drive.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.setPowers(0);

        switch (coneDetection.getPosition()) {
            case RED:
                arm.grab();
                drive.drive(-3120, -3120, 0.5f);
                arm.ungrab();
                drive.drive(straight, -straight, 0.5f);
                drive.drive(-1000, 1000, 0.25f);
                drive.drive(-3000, -3000, 0.5f);
                arm.grab();
                drive.drive(1000, 1000, 0.5f);
                break;
            case BLUE:
                arm.grab();
                drive.drive(-3120, -3120, 0.5f);
                arm.ungrab();
                drive.drive(straight, -straight, 0.5f);
                drive.drive(1000, -1000, 0.25f);
                drive.drive(-3000, -3000, 0.5f);
                arm.grab();
                drive.drive(1000, 1000, 0.5f);
                break;
        }
        requestOpModeStop();
    }

    @Override
    public void update() {
    }

    @Override
    public void onStop() {
    }
}
