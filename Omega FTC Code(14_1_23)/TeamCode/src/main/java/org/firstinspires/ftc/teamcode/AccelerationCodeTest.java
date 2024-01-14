package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Acceleration3", group = "Autonomous")
public class AccelerationCodeTest extends LinearOpMode {

    private Encoder rightEncoder;
    private Encoder leftEncoder;

    private double targetDistanceInInches = 40.0; // Adjust the target distance in inches
    private double maxSpeed = 0.7; // Maximum speed (power level)
    private  double accelerationRate=0.01;

    private double decelerationRate=0.05;

    private double countsPerInch = 1917; // Adjust for your robot's configuration



    //Function to move the robot forward
    public void forward(){

        SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);


        leftEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
        rightEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));


        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculating target distance in ticks per inch
        double targetEncoderValue = (double) (targetDistanceInInches * countsPerInch);
        double currentEncoderValue = leftEncoder.getCurrentPosition();
        double currentSpeed = 0.0;


        while (currentEncoderValue < targetEncoderValue) {

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

            drive.setMotorPowers(currentSpeed,currentSpeed,currentSpeed,currentSpeed);

            currentEncoderValue=leftEncoder.getCurrentPosition();

        }
    }

    // Function for backward motion
    public void backward(){

        SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);

        leftEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
        rightEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculating target distance in ticks per inch
        double targetEncoderValue =  -(double) (targetDistanceInInches * countsPerInch);
        double leftEncodervalue=leftEncoder.getCurrentPosition();
        double rightEncodervalue=rightEncoder.getCurrentPosition();

        double currentEncoderValue=(leftEncodervalue+rightEncodervalue)/2;
        double currentSpeed = 0.0;


        while (currentEncoderValue > targetEncoderValue) {

            if (currentEncoderValue > targetEncoderValue*0.2) {
                // Accelerate
                currentSpeed -= accelerationRate;
            } else if ((currentEncoderValue < (targetEncoderValue*0.2)) && (currentEncoderValue > (targetEncoderValue-(targetEncoderValue*0.2)))) {
                // Maintain constant velocity
                currentSpeed = -maxSpeed;
            } else {
                // Decelerate
                currentSpeed += decelerationRate;
                if(currentSpeed>0.0 || currentEncoderValue<targetEncoderValue){
                    currentSpeed=0;
                }
            }
            // Limit the speed to the maximum
            if (currentSpeed < -maxSpeed) {
                currentSpeed = -maxSpeed;
            }

            drive.setMotorPowers(currentSpeed,currentSpeed,currentSpeed,currentSpeed);

            currentEncoderValue=leftEncoder.getCurrentPosition();

        }
    }


    @Override
    public void runOpMode() {

        waitForStart();

        SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);

        while(opModeIsActive()){

            sleep(500);
            forward();
            sleep(500);
            backward();
            sleep(500);

        }

        // Stop all four motors when the motion is complete
        drive.setMotorPowers(0,0,0,0);
        return;
    }
}
