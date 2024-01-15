package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Scanner;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Acceleration2", group = "Autonomous")
public class AccelerationCode extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private Encoder rightEncoder;
    private Encoder leftEncoder;

    private double targetDistanceInInches = 50.0; // Adjust the target distance in inches
    private double maxSpeed = 0.7; // Maximum speed (power level)
    private  double accelerationRate=0.01;

    private double decelerationRate=0.05;

    private double countsPerInch = 1917; // Adjust for your robot's configuration


    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

        leftEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
        rightEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));

        // Set motor directions if needed
        rightRear.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        double targetEncoderValue = (double) (targetDistanceInInches * countsPerInch);
        double currentEncoderValue = leftEncoder.getCurrentPosition();
        double currentSpeed = 0.0;

        waitForStart();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive() && currentEncoderValue < targetEncoderValue) {

            if (currentEncoderValue < targetEncoderValue*0.2) {
                // Accelerate
                currentSpeed += accelerationRate;
            } else if ((currentEncoderValue > (targetEncoderValue*0.2)) && (currentEncoderValue < (targetEncoderValue-(targetEncoderValue*0.2)))) {
                // Maintain constant velocity
                currentSpeed = maxSpeed;
            } else {
                // Decelerate
                currentSpeed -= decelerationRate;
                if(currentSpeed<0.0 || currentEncoderValue>targetEncoderValue){
                    currentSpeed=0;
                }
            }
            // Limit the speed to the maximum
            if (currentSpeed > maxSpeed) {
                currentSpeed = maxSpeed;
            }

            // Set power for all four motors
            leftFront.setPower(currentSpeed);
            leftRear.setPower(currentSpeed);
            rightRear.setPower(currentSpeed);
            rightFront.setPower(currentSpeed);

            currentEncoderValue=leftEncoder.getCurrentPosition();

        }

        // Stop all four motors when the motion is complete
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
        rightFront.setPower(0.0);
    }
}



