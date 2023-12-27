package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="colorDetection")
public class ColorDetectionHSVExample extends LinearOpMode {


    private ColorSensor colorSensor;

    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // Initialize hardware
        initializeHardware();

        // Wait for the start button to be pressed
        waitForStart();

        // Run the color detection logic
        while (opModeIsActive()) {
            // Get HSV values from the color sensor
            float[] hsvValues = new float[3];
            android.graphics.Color.RGBToHSV(
                    colorSensor.red() * 255,
                    colorSensor.green() * 255,
                    colorSensor.blue() * 255,
                    hsvValues
            );

            // Get distance from the distance sensor
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            // Display HSV values on telemetry
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Saturation", hsvValues[1]);
            telemetry.addData("Value", hsvValues[2]);
            telemetry.addData("Distance", distance);
            telemetry.update();

            // Detect color based on HSV values
            Color detectedColor = detectColor(hsvValues);

            // Perform actions based on the detected color
            switch (detectedColor) {
                case RED:
                    // Do something for red color
                    telemetry.addData("Detected Color", "Red");
                    break;
                case BLUE:
                    // Do something for blue color
                    telemetry.addData("Detected Color", "Blue");
                    break;
                case GREEN:
                    // Do something for green color
                    telemetry.addData("Detected Color", "Green");
                    break;
                case NONE:
                    // Do something when no specific color is detected
                    telemetry.addData("Detected Color", "None");
                    break;
            }

            telemetry.update();
        }
    }

    private void initializeHardware() {
        // Initialize color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        distanceSensor=hardwareMap.get(DistanceSensor.class,"color_sensor");

    }

    private Color detectColor(float[] hsvValues) {
        // Add your color detection logic here based on HSV values
        // You may need to adjust these threshold values based on your specific conditions

        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        // Define HSV thresholds for each color
        float redHueThreshold1 = 0;
        float redHueThreshold2 = 30;     // Adjust these values based on your specific conditions
        float blueHueThreshold1 = 200;
        float blueHueThreshold2 = 250;
        float greenHueThreshold1 = 60;
        float greenHueThreshold2 = 150;

        if ((hue >= redHueThreshold1 && hue <= redHueThreshold2)) {
            return Color.RED;
        } else if (hue >= blueHueThreshold1 && hue <= blueHueThreshold2) {
            return Color.BLUE;
        } else if (hue >= greenHueThreshold1 && hue <= greenHueThreshold2) {
            return Color.GREEN;
        } else {
            return Color.NONE;
        }
    }

    // Enum to represent possible detected colors
    private enum Color {
        RED, BLUE, GREEN, NONE
    }
}
