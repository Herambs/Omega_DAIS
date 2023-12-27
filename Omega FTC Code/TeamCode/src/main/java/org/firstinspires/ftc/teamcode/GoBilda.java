package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="GoBildaRudy")
public class GoBilda extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private DcMotor intake;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
//        intake = hardwareMap.dcMotor.get("intake");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
//        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Gamepad controls
        double leftStickY = gamepad1.left_stick_y; // Reverse Y-axis for natural forward/backward motion
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        // Calculate motor powers for holonomic drive
        double frontLeftPower = leftStickY - leftStickX + rightStickX;
        double frontRightPower = leftStickY + leftStickX - rightStickX;
        double backLeftPower = leftStickY + leftStickX + rightStickX;
        double backRightPower = leftStickY - leftStickX - rightStickX;

        // Set motor powers for holonomic drive
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Display runtime and motor powers on telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.update();

//        boolean Button=gamepad1.x;
//
//        if(Button){
//            intake.setPower(0.5);
//        }else {
//            intake.setPower(0);
//        }
    }
}
