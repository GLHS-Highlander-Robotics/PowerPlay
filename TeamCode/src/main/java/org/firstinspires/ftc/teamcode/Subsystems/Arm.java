package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class Arm implements Subsystem {
    private final RobotOpMode opMode;
    private final int armMotorSteps = 0;
    private final double gripPos = 1;
    private final boolean dpadpressed = false;
    public int armMax;
    public int armMin;
    public double gripMax;
    public double gripMin;
    public DcMotor armMotor;
    public Servo leftGripper;
    public Servo rightGripper;

    public Arm(RobotOpMode opMode) {
        this.opMode = opMode;
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
