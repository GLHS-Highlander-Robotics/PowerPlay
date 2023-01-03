package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;

public class RearTankDrive implements Subsystem {
    private final RobotOpMode opMode;
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public int leftPos = 0;
    public int rightPos = 0;

    public RearTankDrive(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void setup() {
        leftMotor = opMode.hardwareMap.get(DcMotor.class, "b_left");
        rightMotor = opMode.hardwareMap.get(DcMotor.class, "b_right");
    }

    @Override
    public void onStart() {
    }

    @Override
    public void update() {
    }

    public void drive(int leftmove, int rightmove, float speed) {
        leftPos += leftmove;
        rightPos += rightmove;
        leftMotor.setTargetPosition(leftPos);
        rightMotor.setTargetPosition(rightPos);
        setModes(DcMotor.RunMode.RUN_TO_POSITION);
        setPowers(speed);
        opMode.blockOn(leftMotor, rightMotor);
    }

    public void setModes(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    public void setPowers(float speed) {
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
    }
}
