package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class OnBot extends LinearOpMode {

    private DcMotor leftFront = hardwareMap.get(DcMotor.class, "left_front");

    private DcMotor leftRear= hardwareMap.get(DcMotor.class, "left_back");

    private  DcMotor rightRear=hardwareMap.get(DcMotor.class,"right_back");

    private DcMotor rightFront=hardwareMap.get(DcMotor.class,"right_front");

    private DcMotor leftEncoder = hardwareMap.get(DcMotor.class, "left_back");
    private DcMotor rightEncoder = hardwareMap.get(DcMotor.class, "left_front");
    private DcMotor frontEncoder = hardwareMap.get(DcMotor.class, "right_back");


    private double targetDistanceInInches;

    private double targetAngle;

    private double countsPerInch = 1917; // Adjust for your robot's configuration

    public void forward(double distance){

        //Initializing the motors and the encoders
//        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
//        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
//        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_back"));

        //Setting Directions for motors and encoders
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

        //Reseting the Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetDistanceInInches=distance;

        double targetEncoderValue= (targetDistanceInInches*countsPerInch);

        double currentEncoderValue=leftEncoder.getCurrentPosition();

        while(currentEncoderValue<targetEncoderValue){

            leftFront.setPower(0.6);
            leftRear.setPower(0.6);
            rightRear.setPower(0.6);
            rightFront.setPower(0.6);

            double leftEncoderValue=leftEncoder.getCurrentPosition();
            double rightEncoderValue=rightEncoder.getCurrentPosition();

            currentEncoderValue=(leftEncoderValue+rightEncoderValue)/2;

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    public void backward(double backwardDistance){

        //Setting Directions for motors and encoders
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

        //Reseting the Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetDistanceInInches=backwardDistance;

        double targetEncoderValue= -(targetDistanceInInches*countsPerInch);

        double currentEncoderValue=leftEncoder.getCurrentPosition();

        while(currentEncoderValue>targetEncoderValue){

            leftFront.setPower(-0.6);
            leftRear.setPower(-0.6);
            rightRear.setPower(-0.6);
            rightFront.setPower(-0.6);

            double leftEncoderValue= -leftEncoder.getCurrentPosition();
            double rightEncoderValue= -rightEncoder.getCurrentPosition();

            currentEncoderValue=(leftEncoderValue+rightEncoderValue)/2;

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeRight(double strafeRightDistance){

        //Initializing the motors and Encoders
//        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
//        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
//        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
//
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_back"));

        //Setting Directions for motors and Encoders
        leftFront.setDirection(DcMotor.Direction.REVERSE); // add if needed
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

        //Reseting the Encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetDistanceInInches=strafeRightDistance; // Target Distance in inches

        double targetEncoderValue= -(targetDistanceInInches*countsPerInch);

        double currentEncoderValue=frontEncoder.getCurrentPosition();

        while(currentEncoderValue>targetEncoderValue){

            leftFront.setPower(0.6);
            leftRear.setPower(0.6);
            rightRear.setPower(0.6);
            rightFront.setPower(0.6);

            currentEncoderValue=frontEncoder.getCurrentPosition();

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeLeft(double strafeLeftDistance){

        //Setting Directions for motors and Encoders
        leftFront.setDirection(DcMotor.Direction.FORWARD); // add if needed
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

        //Reseting the Encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetDistanceInInches=strafeLeftDistance; // Target Distance in inches

        double targetEncoderValue= (targetDistanceInInches*countsPerInch);

        double currentEncoderValue=frontEncoder.getCurrentPosition();

        while(currentEncoderValue<targetEncoderValue){

            leftFront.setPower(0.6);
            leftRear.setPower(0.6);
            rightRear.setPower(0.6);
            rightFront.setPower(0.6);

            currentEncoderValue=frontEncoder.getCurrentPosition();

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

    }

    public void turnRight(double rightAngle){

        //Initializing the motors and the encoders
//        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
//        leftRear = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightRear = hardwareMap.get(DcMotorEx.class, "right_back");
//        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
//
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_back"));

        //Setting Directions for motors and Encoders
        leftFront.setDirection(DcMotor.Direction.REVERSE); // add if needed
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

        targetAngle=rightAngle;

        double angleEncoderValue= (targetAngle/360)*8192;

        double currentEncoderValue=frontEncoder.getCurrentPosition();

        while(currentEncoderValue<angleEncoderValue){

            leftFront.setPower(0.6);
            leftRear.setPower(0.6);
            rightRear.setPower(0.6);
            rightFront.setPower(0.6);

            currentEncoderValue=frontEncoder.getCurrentPosition();

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    public void turnLeft(double leftAngle){

        leftFront.setDirection(DcMotor.Direction.FORWARD); // add if needed
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

        targetAngle=leftAngle;

        double angleEncoderValue= -(targetAngle/360)*8192;

        double currentEncoderValue=frontEncoder.getCurrentPosition();

        while(currentEncoderValue>angleEncoderValue){

            leftFront.setPower(0.6);
            leftRear.setPower(0.6);
            rightRear.setPower(0.6);
            rightFront.setPower(0.6);

            currentEncoderValue=frontEncoder.getCurrentPosition();

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {


    }
}
