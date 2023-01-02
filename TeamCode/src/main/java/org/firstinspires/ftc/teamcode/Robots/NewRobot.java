package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.StrafeDrive;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;

public class NewRobot {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Hardware Configuration Names
    public static final String FRONT_LEFT_MOTOR_NAME = "motor_front_left";
    public static final String FRONT_RIGHT_MOTOR_NAME = "motor_front_right";
    public static final String BACK_LEFT_MOTOR_NAME = "motor_back_left";
    public static final String BACK_RIGHT_MOTOR_NAME = "motor_back_right";
//    public static final String LINEAR_SLIDE_MOTOR_NAME = "motor_slide";
//    public static final String GRIPPER1_SERVO_NAME = "grip1";
//    public static final String GRIPPER2_SERVO_NAME = "grip2";

    // Subsystems
    public final StrafeDrive strafeDrive;
    public final LinearOpMode linearOpMode;
//    public final LinearSlide linearSlide;

    // Sensors
    public final Gamepad gamepad;

    public NewRobot(LinearOpMode linearOpMode, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.gamepad = gamepad;
        this.strafeDrive = initStrafeDrive();
//        this.linearSlide = initLinearSlide();
    }
    
    public StrafeDrive initStrafeDrive() {
        DcMotor frontLeftMotor = (DcMotor) hardwareMap.get(FRONT_LEFT_MOTOR_NAME);
        DcMotor frontRightMotor = (DcMotor) hardwareMap.get(FRONT_RIGHT_MOTOR_NAME);
        DcMotor backLeftMotor = (DcMotor) hardwareMap.get(BACK_LEFT_MOTOR_NAME);
        DcMotor backRightMotor = (DcMotor) hardwareMap.get(BACK_RIGHT_MOTOR_NAME);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        return new StrafeDrive(linearOpMode,
                telemetry,
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gamepad
        );
    }

//    public LinearSlide initLinearSlide(){
//        DcMotor slideMotor = (DcMotor) hardwareMap.get(LINEAR_SLIDE_MOTOR_NAME);
//        slideMotor.setTargetPosition(0);
//        slideMotor.setPower(1);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        Servo gripper1Servo = (Servo) hardwareMap.get(GRIPPER1_SERVO_NAME);
//        Servo gripper2Servo = (Servo) hardwareMap.get(GRIPPER2_SERVO_NAME);
//        return new LinearSlide(
//                linearOpMode,
//                telemetry,
//                slideMotor,
//                gripper1Servo,
//                gripper2Servo,
//                gamepad,
//                0,
//                450,
//                0.45,
//                1
//                );
//    }
}
