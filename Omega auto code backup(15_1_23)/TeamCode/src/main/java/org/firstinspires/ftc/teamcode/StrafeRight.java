package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="StrafeRight")
public class StrafeRight extends LinearOpMode {

    private double targetDistanceInInches = 36.0; // Adjust the target distance in inches

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor rightFront;

    private Encoder rightEncoder;
    private Encoder leftEncoder;

    private Encoder frontEncoder;

    private double countsPerInch = 1917; // Adjust for your robot's configuration
    private double targetEncoderValue = -(double) (targetDistanceInInches * countsPerInch);


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_back"));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double frontEncoderValue=frontEncoder.getCurrentPosition();

        while(frontEncoderValue>targetEncoderValue){

            leftFront.setPower(0.7);
            leftRear.setPower(0.7);
            rightRear.setPower(0.7);
            rightFront.setPower(0.7);

            frontEncoderValue=frontEncoder.getCurrentPosition();

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
        return;

    }
}
