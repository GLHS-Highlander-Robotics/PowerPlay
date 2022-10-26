package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.BackTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.SingleJointGripperArm;

public class OldRobot {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Hardware Configuration Names
    public static final String BACK_LEFT_MOTOR_NAME = "b_left";
    public static final String BACK_RIGHT_MOTOR_NAME = "b_right";
    public static final String ARM_MOTOR_NAME = "arm";
    public static final String GRIPPER1_SERVO_NAME = "grip1";
    public static final String GRIPPER2_SERVO_NAME = "grip2";

    // Subsystems
    public final BackTankDrive backTankDrive;
    public final SingleJointGripperArm singleJointGripperArm;

    // Sensors
    public final Gamepad gamepad;

    public OldRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.backTankDrive = initBackTankDrive();
        this.singleJointGripperArm = initSingleJointGripperArm();
    }

    public BackTankDrive initBackTankDrive() {
        DcMotor backLeftMotor = (DcMotor) hardwareMap.get(BACK_LEFT_MOTOR_NAME);
        DcMotor backRightMotor = (DcMotor) hardwareMap.get(BACK_RIGHT_MOTOR_NAME);

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        return new BackTankDrive(
                telemetry,
                backLeftMotor,
                backRightMotor,
                gamepad,
                100,
                5000, // TODO: get what this value is in class
                0.1 // TODO: measure in class what this value is
        );
    }

    public SingleJointGripperArm initSingleJointGripperArm() {
        DcMotor armMotor = (DcMotor) hardwareMap.get(ARM_MOTOR_NAME);

        armMotor.setTargetPosition(0);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Servo gripper1Servo = (Servo) hardwareMap.get(GRIPPER1_SERVO_NAME);
        Servo gripper2Servo = (Servo) hardwareMap.get(GRIPPER2_SERVO_NAME);

        return new SingleJointGripperArm(
                telemetry,
                armMotor,
                gripper1Servo,
                gripper2Servo,
                gamepad,
                0.45,
                1,
                0,
                450
        );
    }
}
