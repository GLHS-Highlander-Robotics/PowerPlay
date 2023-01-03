package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class LinearSlide implements Subsystem {
    public final int minHeight;
    public final int maxHeight;
    public final double gripMin;
    public final double gripMax;
    private final RobotOpMode opMode;
    private final double gripPos = 1;
    private final double height = 0;
    private final int armMotorSteps = 0;
    private final boolean dpadPressed = false;
    public DcMotor slideMotor;
    public Servo leftGripper, rightGripper;

    public LinearSlide(RobotOpMode opMode, int maxHeight, int minHeight, double gripMin, double gripMax) {
        this.opMode = opMode;
        this.maxHeight = maxHeight;
        this.minHeight = minHeight;
        this.gripMin = gripMin;
        this.gripMax = gripMax;
    }

    public void setup() {
        slideMotor = opMode.hardwareMap.get(DcMotor.class, "motor_slide");
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // do we need this line?
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftGripper = opMode.hardwareMap.get(Servo.class, "grip1");
        rightGripper = opMode.hardwareMap.get(Servo.class, "grip2");
    }

    @Override
    public void onStart() {
    }

    public void update() {
    }

    @Override
    public void onStop() {
    }

    public void grab() {
        leftGripper.setPosition(1);
        rightGripper.setPosition(0);
    }

    public void ungrab() {
        leftGripper.setPosition(gripMin);
        rightGripper.setPosition(1 - gripMin);
    }

    public void setSlide(int steps) {
        slideMotor.setTargetPosition(Utils.clamp(steps, minHeight, maxHeight));
        opMode.blockOn(slideMotor);
    }
}
