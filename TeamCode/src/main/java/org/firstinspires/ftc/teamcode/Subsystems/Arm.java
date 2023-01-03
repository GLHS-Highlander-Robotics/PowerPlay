package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.old.Util.Maths;

public class Arm implements Subsystem {
    private final RobotOpMode opMode;
    public int armMin, armMax;
    public double gripMin, gripMax;
    public DcMotor armMotor;
    public Servo leftGripper, rightGripper;
    private double gripPos = 1;
    private boolean dPadPressed = false;
    private int armMotorSteps = 0;

    public Arm(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    public void updateByGamepad() {
        // Arm
        if (opMode.gamepad1.a) {
            armMotorSteps = 0;
        } else if (opMode.gamepad1.b) {
            armMotorSteps = 86;
        } else if (opMode.gamepad1.x) {
            armMotorSteps = 270;
        } else if (opMode.gamepad1.y) {
            armMotorSteps = 460;
        }

        // TODO: check if this sensitivity is right
        if (opMode.gamepad1.dpad_up) {
            armMotorSteps += 4;
            dPadPressed = true;
        } else if (opMode.gamepad1.dpad_down) {
            armMotorSteps -= 4;
            dPadPressed = true;
        } else if (dPadPressed) {
            armMotorSteps = armMotor.getCurrentPosition();
            dPadPressed = false;
        }

        armMotorSteps = Utils.clamp(armMotorSteps, armMin, armMax);

        armMotor.setTargetPosition(armMotorSteps);

        // Gripper

        if (opMode.gamepad1.right_trigger > 0.5) {
            gripPos = gripMin;
        } else if (opMode.gamepad1.left_trigger > 0.5) {
            gripPos = gripMax;
        }

        gripPos = Maths.clamp(gripPos, gripMin, gripMax);

        leftGripper.setPosition(1 - gripPos);
        rightGripper.setPosition(gripPos);
    }

    @Override
    public void setup() {
        armMotor = opMode.hardwareMap.get(DcMotor.class, "arm");
        leftGripper = opMode.hardwareMap.get(Servo.class, "grip1");
        rightGripper = opMode.hardwareMap.get(Servo.class, "grip2");
    }

    @Override
    public void onStart() {
    }

    @Override
    public void update() {
    }

    @Override
    public void onStop() {
    }

    public void grab() {
        leftGripper.setPosition(0);
        rightGripper.setPosition(1);
    }

    public void ungrab() {
        leftGripper.setPosition(1 - gripMin);
        rightGripper.setPosition(gripMin);
    }

    public void setArm(int steps) {
        armMotor.setTargetPosition(Utils.clamp(steps, armMin, armMax));
        opMode.blockOn(armMotor);
    }
}
