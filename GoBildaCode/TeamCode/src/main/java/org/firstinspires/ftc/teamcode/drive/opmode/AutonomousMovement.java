package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousMovement", group = "Autonomous")
public class AutonomousMovement extends LinearOpMode {

    private double forwardDistance=0;

    private double backwardDistance=0;

    private double rightAngle=0;

    private double leftAngle=0;

    private double strafeRightDistance=0;

    private double strafeLeftDistance=0;

    private double countsPerInch=1917.0;


    public void forward(double distance){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        forwardDistance=distance;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(forwardDistance)
                .build();

        drive.followTrajectory(forwardTrajectory);

    }

//    public void variableForward(double distance){
//
//         double accelerationRate=0.01;
//
//         double decelerationRate=0.04;
//
//         double maxSpeed=0.65;
//
//         double minSpeed=0.2;
//
//        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);
//
//        leftEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
//
//        forwardDistance=distance;
//
//        double targetEncoderValue= forwardDistance*countsPerInch;
//        double currentEncoderValue=leftEncoder.getCurrentPosition();
//        double currentSpeed=0.0;
//
//        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        while(currentEncoderValue<targetEncoderValue){
//
//            if (currentEncoderValue < targetEncoderValue*0.2) {
//                // Accelerate
//                currentSpeed += accelerationRate;
//            } else if ((currentEncoderValue > (targetEncoderValue*0.2)) && (currentEncoderValue < (targetEncoderValue-(targetEncoderValue*0.2)))) {
//                // Maintain constant velocity
//                    currentSpeed = maxSpeed;
//            } else {
//                // Decelerate
//                if(currentEncoderValue>targetEncoderValue){
//                    currentSpeed=0;
//                }else if(currentSpeed<0.0 && currentEncoderValue<targetEncoderValue){
//                    currentSpeed=minSpeed;
//                }else {
//                    currentSpeed -= decelerationRate;
//                }
//            }
//            // Limit the speed to the maximum
//            if (currentSpeed > maxSpeed) {
//                currentSpeed = maxSpeed;
//            }
//
//            drive.setMotorPowers(currentSpeed,currentSpeed,currentSpeed,currentSpeed);
//
//            currentEncoderValue=leftEncoder.getCurrentPosition();
//        }
//
//        drive.setMotorPowers(0,0,0,0);
//        return;
//
//    }

    public void backward(double distance){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        backwardDistance=distance;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory backwardTrajectory = drive.trajectoryBuilder(startPose)
                .back(backwardDistance)
                .build();

        drive.followTrajectory(backwardTrajectory);

    }

    public void turnRight(double turnAngle){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        rightAngle=turnAngle;

        drive.turn(Math.toRadians(-rightAngle));
    }

    public void turnLeft(double turnAngle){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        leftAngle=turnAngle;

        drive.turn(Math.toRadians(leftAngle));

    }

    public void strafeRight(double strafeRightDis){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        strafeRightDistance=strafeRightDis;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory strafeRightTrajectory = drive.trajectoryBuilder(startPose)
                .strafeRight(strafeRightDistance)
                .build();

        drive.followTrajectory(strafeRightTrajectory);

    }

    public void strafeLeft(double strafeLeftDis){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        strafeLeftDistance=strafeLeftDis;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory strafeLeftTrajectory = drive.trajectoryBuilder(startPose)
                .strafeLeft(strafeLeftDistance)
                .build();

        drive.followTrajectory(strafeLeftTrajectory);

    }
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if(isStopRequested()){
            return;
        }
        auto.servoZero();
        forward(48.0);
        turnLeft(90);
        forward(75.0);
        strafeLeft(27.0);  // center strafe
        auto.channelMotion(0.5);
        sleep(1500);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        forward(3.5);
        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.3);
        sleep(1200);
        auto.channelZero();

//        forward(15.0);
//        turnRight(90.0);
//        forward(62.0);
//        turnRight(90.0);
//        forward(15.0);
//        turnLeft(90.0);
//        forward(10.0);

    }
}
