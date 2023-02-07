package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.Utils;

public class StrafeDrive implements Subsystem {
    private final RobotOpMode opMode;
    public IMU imu;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;
    private double botHeading;
    private double limiter = 0.75;
    private boolean field = false;

    public StrafeDrive(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void setup() {
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor_back_right");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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

        backRightMotor.setTargetPosition(brPos);
        frontLeftMotor.setTargetPosition(flPos);
        backLeftMotor.setTargetPosition(blPos);
        frontRightMotor.setTargetPosition(frPos);


        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
//        while (opMode.opModeIsActive() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
//            opMode.idle();
//        }
        opMode.blockOn(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }


    public void strafe(int move, float speed) {

        flPos += move;
        frPos += move;
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


    public void turnLeft(double rad, float speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        while (botHeading < rad) {

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            opMode.telemetry.addData("Heading: ", botHeading);

            opMode.telemetry.update();
            opMode.idle();

        }
        setPowers(0);
        flPos = frontLeftMotor.getCurrentPosition();
        frPos = frontRightMotor.getCurrentPosition();
        blPos = backLeftMotor.getCurrentPosition();
        brPos = backRightMotor.getCurrentPosition();

    }

    public void turnRight(double rad, float speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        while (botHeading > rad) {

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            opMode.telemetry.addData("Heading: ", botHeading);

            opMode.telemetry.update();
            opMode.idle();

        }
        setPowers(0);
        flPos = frontLeftMotor.getCurrentPosition();
        frPos = frontRightMotor.getCurrentPosition();
        blPos = backLeftMotor.getCurrentPosition();
        brPos = backRightMotor.getCurrentPosition();

    }
    public void updateByTwoGamepads(){

        double forward;
        double strafe;
        double rotate;
        double fieldForward;
        double fieldStrafe;

        if(opMode.gamepad1.a){
            limiter = 0.75;
        }else if(opMode.gamepad1.b){
            limiter = 0.25;
        }
        if(opMode.gamepad1.x){

            imu.resetYaw();
            field = true;
        }else if(opMode.gamepad1.y){
            field = false;
        }


        forward = -opMode.gamepad1.left_stick_y * limiter;
        strafe = opMode.gamepad1.left_stick_x * limiter;
        rotate = opMode.gamepad1.right_stick_x * limiter;

        if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
            forward = -opMode.gamepad2.left_stick_y*0.4;
            if(Math.abs(opMode.gamepad2.left_stick_y)<0.02){
                forward=0;
            }
        }
        if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
            strafe = 0;
            strafe = opMode.gamepad2.left_stick_x*0.4;
            if(Math.abs(opMode.gamepad2.left_stick_x)<0.02){
                strafe=0;
            }
        }
        if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
            rotate = opMode.gamepad2.right_stick_x * 0.4;
            if(Math.abs(opMode.gamepad2.right_stick_x)<0.02){
                rotate=0;
            }
        }
        fieldForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        fieldStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (!field) {
            frontLeftMotor.setPower((forward + strafe + rotate));
            backLeftMotor.setPower((forward - strafe + rotate));
            frontRightMotor.setPower((forward + strafe - rotate));
            backRightMotor.setPower((forward - strafe - rotate));
        } else {
            frontLeftMotor.setPower((fieldForward + fieldStrafe + rotate));
            backLeftMotor.setPower((fieldForward - fieldStrafe + rotate));
            frontRightMotor.setPower((fieldForward + fieldStrafe - rotate));
            backRightMotor.setPower((fieldForward - fieldStrafe - rotate));
        }
        opMode.telemetry.addData("Limiter: ", limiter);
        opMode.telemetry.addData("Heading: ", botHeading);
        opMode.telemetry.addData("Field Centric?: ", field);
        opMode.telemetry.update();

    }

    public void updateByGamepad() {
        double forward;
        double strafe;
        double rotate;
//        if (Math.abs(-opMode.gamepad2.left_stick_y) >= 0.01 || Math.abs(opMode.gamepad1.left_stick_x) >= 0.01 || Math.abs(opMode.gamepad1.right_stick_x) >= 0.01) {
//            slow = true;
//        } else {
//            slow = false;
//        }
//        if (slow) {
//            forward = -opMode.gamepad2.left_stick_y * (limiter / 2);
//            strafe = opMode.gamepad2.left_stick_x * (limiter / 2);
//            rotate = opMode.gamepad2.right_stick_x * (limiter / 2);
//            if (Math.abs(-opMode.gamepad2.left_stick_y) < 0.01) {
//                forward = 0;
//            }
//            if (Math.abs(opMode.gamepad2.left_stick_x) < 0.01) {
//                strafe = 0;
//            }
//            if (Math.abs(opMode.gamepad2.right_stick_x) < 0.01) {
//                rotate = 0;
//            }
//        } else {
            forward = -opMode.gamepad1.left_stick_y * limiter;
            strafe = opMode.gamepad1.left_stick_x * limiter;
            rotate = opMode.gamepad1.right_stick_x * limiter;
            if (Math.abs(-opMode.gamepad1.left_stick_y) < 0.01) {
                forward = 0;
            }
            if (Math.abs(opMode.gamepad1.left_stick_x) < 0.01) {
                strafe = 0;
            }
            if (Math.abs(opMode.gamepad1.right_stick_x) < 0.01) {
                rotate = 0;
            }

        frontLeftMotor.setPower((forward + strafe + rotate));
        backLeftMotor.setPower((forward - strafe + rotate));
        frontRightMotor.setPower((forward + strafe - rotate));
        backRightMotor.setPower((forward - strafe - rotate));
        opMode.telemetry.addData("Limiter: ", limiter);
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
