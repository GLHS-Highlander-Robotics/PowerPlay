package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.RobotOpMode;

abstract public class Subsystem {
    protected final RobotOpMode opMode;

    public Subsystem(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    abstract public void setup();
}