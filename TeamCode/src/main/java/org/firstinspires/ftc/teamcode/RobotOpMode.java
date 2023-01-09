package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.lang.reflect.Field;

abstract public class RobotOpMode extends LinearOpMode {
    private void setupSubsystems() {
        for (Field field : this.getClass().getDeclaredFields()) {
            field.setAccessible(true);
            try {
                Object obj = field.get(this);
                if (obj instanceof Subsystem) {
                    Subsystem subsystem = (Subsystem) obj;
                    subsystem.setup();
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void runOpMode() {
        setupSubsystems();
        main();
    }

    public abstract void main();

    // Waits until at least one device or opmode is not busy
    public void waitFor(DcMotor... motors) {
        while (opModeIsActive() && isBusy(motors)) {
            idle();
        }
    }

    public boolean isBusy(DcMotor... motors) {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }
}
