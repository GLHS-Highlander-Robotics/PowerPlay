package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class StrafeDrive implements Subsystem {
    private final RobotOpMode opMode;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;
    private double limiter = 0.75;

    public StrafeDrive(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void setup() {
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_right");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(int leftMove, int rightMove, float speed) {

        flPos += leftMove;
        frPos += rightMove;
        blPos += leftMove;
        brPos += rightMove;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void driveInches(double leftInches, double rightInches, float speed) {

        flPos += Utils.driveTicks(leftInches);
        frPos += Utils.driveTicks(rightInches);
        blPos += Utils.driveTicks(leftInches);
        brPos += Utils.driveTicks(rightInches);

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
//        while (opMode.opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
//            opMode.idle();
//        }
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void strafe(int move, float speed) {

        flPos = move;
        frPos = move;
        blPos -= move;
        brPos -= move;

        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void strafeInches(double inches, float speed) {

        flPos += Utils.strafeTicks(inches);
        frPos += Utils.strafeTicks(inches);
        blPos -= Utils.strafeTicks(inches);
        brPos -= Utils.strafeTicks(inches);
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);
        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void updateByGamepad() {
        double forward = -opMode.gamepad1.left_stick_y;
        double strafe = opMode.gamepad1.left_stick_x;
        double rotate = opMode.gamepad1.right_stick_x;

        if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
            forward = 0;
        }
        if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
            strafe = 0;
        }
        if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
            rotate = 0;
        }

        if (opMode.gamepad1.left_stick_button) {
            limiter = 0.2;

        } else if (opMode.gamepad1.right_stick_button) {
            limiter = 0.75;
        }

        frontLeftMotor.setPower((forward + strafe + rotate) * limiter);
        backLeftMotor.setPower((forward - strafe + rotate) * limiter);
        frontRightMotor.setPower((forward + strafe - rotate) * limiter);
        backRightMotor.setPower((forward - strafe - rotate) * limiter);
        opMode.telemetry.addData("Speed:", opMode.gamepad1.left_stick_y);
        opMode.telemetry.addData("Limiter", limiter);
        opMode.telemetry.update();
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
