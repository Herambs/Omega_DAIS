package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Config
@Autonomous(name="Straight")
public class Straight extends LinearOpMode {

    private Encoder rightEncoder;
    private Encoder leftEncoder;

    private double targetDistanceInInches = 36.0; // Adjust the target distance in inches

    private double countsPerInch = 1917; // Adjust for your robot's configuration
    private double targetEncoderValue = (double) (targetDistanceInInches * countsPerInch);


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);

        leftEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
        rightEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_front"));


        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double leftEncodervalue=leftEncoder.getCurrentPosition();
        double rightEncodervalue=rightEncoder.getCurrentPosition();

        double currentEncodervalue=(leftEncodervalue+rightEncodervalue)/2;

        while(currentEncodervalue<targetEncoderValue){

            drive.setMotorPowers(0.3,0.3,0.3,0.3);

            leftEncodervalue=leftEncoder.getCurrentPosition();
            rightEncodervalue=rightEncoder.getCurrentPosition();

            currentEncodervalue=(leftEncodervalue+rightEncodervalue)/2;

             telemetry.addData("left", leftEncoder.getCurrentPosition());
             telemetry.addData("right", rightEncoder.getCurrentPosition());
             telemetry.update();
        }

        drive.setMotorPowers(0,0,0,0);
        return;

    }
}
