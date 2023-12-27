package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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

    public void centerDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        forward(46.0);  // Center Drop Forward = 48
        turnLeft(91.0);
        forward(74.5);
        auto.StrafeLeftDistance(0.6);
        strafeLeft(7.0);
        auto.channelMotion(0.5);
        sleep(1500);
        auto.channelZero();
        auto.servoPlace();
        sleep(2000);
        auto.ForwardDistance(0.2);
        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.25);
        sleep(1200);
        auto.channelZero();

    }

    public void leftDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);


        auto.servoZero();
        forward(25.0);
        turnRight(90);
        turnLeft(180.0);
        strafeRight(25.0);
        forward(73.5);
        sleep(500);
        auto.StrafeLeftDistance(0.6);
        strafeLeft(11);
        auto.channelMotion(0.5);
        sleep(1600);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
//        forward(3.5);
        auto.ForwardDistance(0.2);

        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.25);
        sleep(1200);
        auto.channelZero();


    }

    public void rightDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        forward(25.0);
        turnLeft(92.0);
        strafeRight(22.0);
        forward(72.5);
        sleep(500);
        auto.StrafeLeftDistance(0.5);
//        strafeLeft(10.0);
        auto.channelMotion(0.5);
        sleep(1600);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);
//
        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.25);
        sleep(1200);
        auto.channelZero();

    }

    public void rightDropShort(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        forward(25.0);
        turnLeft(92.0);
        forward(25);
        sleep(500);
        strafeRight(9);
        auto.channelMotion(0.5);
        sleep(2000);//1600 for longer distance auto
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.25);
        sleep(1200);
        auto.channelZero();

    }

    public void centerDropShort(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        forward(46.0);
        turnLeft(92.0);
        forward(25);
        sleep(500);
        strafeLeft(20);
        auto.channelMotion(0.5);
        sleep(2000);//1600 for longer distance auto
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.25);
        sleep(1200);
        auto.channelZero();

    }

    public void leftDropShort(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        forward(25.0);
        turnLeft(92);
        forward(20);
        sleep(1000);
        forward(7);
        strafeLeft(7);
        auto.channelMotion(0.5);
        sleep(2000);//1600 for longer distance auto
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.25);
        sleep(1200);
        auto.channelZero();

    }
    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();

        if(isStopRequested()){
            return;
        }

//        centerDrop();  // Spike Location center
//        leftDrop();
//        rightDrop();
//        centerDrop();
//        rightDropShort();
//        centerDropShort();
        leftDropShort();


    }
}
