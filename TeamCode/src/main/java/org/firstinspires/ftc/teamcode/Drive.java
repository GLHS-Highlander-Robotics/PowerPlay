package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive implements Subsystem {
    private final RobotOpMode opMode;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;

    public Drive(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    public void setup() {
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_right");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        // respond to gamepad and whatnot
    }

    public void drive(int leftMove, int rightMove, float speed) {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = leftMove;
        frPos = rightMove;
        blPos = leftMove;
        brPos = rightMove;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);

        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void driveIn(double leftInches, int rightInches, float speed) {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flPos = Utils.sTicks(leftInches);
        frPos = Utils.sTicks(rightInches);
        blPos = Utils.sTicks(leftInches);
        brPos = Utils.sTicks(rightInches);

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);

        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void setModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    public void setPowers(float speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }
}
