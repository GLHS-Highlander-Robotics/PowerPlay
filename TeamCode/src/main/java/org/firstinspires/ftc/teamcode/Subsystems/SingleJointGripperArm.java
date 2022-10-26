package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Maths;

public class SingleJointGripperArm {
    private final Telemetry telemetry;
    public final DcMotor armMotor;
    public final Servo gripper1Servo;
    public final Servo gripper2Servo;
    public final Gamepad gamepad;
    public final float gripMin;
    public final float gripMax;
    public final int armMin;
    public final int armMax;
    private int armMotorSteps = 0;
    private float gripPos = 1;

    public SingleJointGripperArm(Telemetry telemetry, DcMotor armMotor, Servo gripper1Servo, Servo gripper2Servo, Gamepad gamepad, float gripMin, float gripMax, int armMin, int armMax) {
        this.telemetry = telemetry;
        this.armMotor = armMotor;
        this.gripper1Servo = gripper1Servo;
        this.gripper2Servo = gripper2Servo;
        this.gamepad = gamepad;
        this.gripMin = gripMin;
        this.gripMax = gripMax;
        this.armMin = armMin;
        this.armMax = armMax;
    }

    public void respondToGamepad() {
        armRespondToGamepad();
        gripRespondToGamepad();
    }

    public void gripRespondToGamepad() {
        if (gamepad.right_trigger > 0.5) {
            gripPos = gripMin;
        } else if (gamepad.left_trigger > 0.5) {
            gripPos = gripMax;
        } else if (gamepad.right_bumper) {
            gripPos -= 0.1;
            sleep(200);
        } else if (gamepad.left_bumper) {
            gripPos += 0.1;
            sleep(200);
        }

        gripPos = Maths.clamp(gripPos, gripMin, gripMax);

        gripper1Servo.setPosition(1 - gripPos);
        gripper2Servo.setPosition(gripPos);
    }

    public void armRespondToGamepad() {
        if (gamepad.a) {
            armMotorSteps = 0;
        } else if (gamepad.b) {
            armMotorSteps = 86;
        } else if (gamepad.x) {
            armMotorSteps = 290;
        } else if (gamepad.y) {
            armMotorSteps = 430;
        }

        // TODO: check if this sensitivity is right
        if (gamepad.dpad_up) {
            armMotorSteps++;
        } else if (gamepad.dpad_down) {
            armMotorSteps--;
        }

        armMotorSteps = Maths.clamp(armMotorSteps, armMin, armMax);

        armMotor.setTargetPosition(armMotorSteps);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}