package org.firstinspires.ftc.teamcode.subsystem.colorsensor;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Config
public class ColorSensor {
    private final NormalizedColorSensor colorSensor;
    private NormalizedRGBA colors;
    private float[] hsvValues = new float[3];

    public float gain = 2;

    public ColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(gain);
    }

    public void update() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
    }

    public float getHue() {
        update();
        return hsvValues[0];
    }

    public float getSaturation() {
        update();
        return hsvValues[1];
    }

    public float getValue() {
        update();
        return hsvValues[2];
    }

    public float getRed() {
        update();
        return colors.red;
    }

    public float getBlue() {
        update();
        return colors.blue;
    }

    public float getGreen() {
        update();
        return colors.green;
    }

    public float getAlpha() {
        update();
        return colors.alpha;
    }

    public float getGain() {
        update();
        return colorSensor.getGain();
    }
}