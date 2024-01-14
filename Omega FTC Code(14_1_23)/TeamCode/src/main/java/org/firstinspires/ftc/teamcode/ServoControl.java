package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ServoControl extends OpMode {

    private Servo grabberServo;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Initialize servo
        grabberServo = hardwareMap.servo.get("grabber_servo");

        // Set initial servo position
        grabberServo.setPosition(0.0); // Adjust as needed
    }

    @Override
    public void loop() {
        // Gamepad controls
        if (gamepad1.a) {
            // 'A' button pressed, close the grabber (adjust position as needed)
            grabberServo.setPosition(0.5);
        } else if (gamepad1.b) {
            // 'B' button pressed, open the grabber (adjust position as needed)
            grabberServo.setPosition(0.0);
        }

        // Display runtime and servo position on telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Servo Position", grabberServo.getPosition());
        telemetry.update();
    }
}
