package org.firstinspires.ftc.teamcode.old.subsystem;

import org.firstinspires.ftc.teamcode.old.RobotOpMode;

abstract public class Subsystem {
    protected final RobotOpMode opMode;

    public Subsystem(RobotOpMode opMode) {
        this.opMode = opMode;
    }

    abstract public void setup();
}