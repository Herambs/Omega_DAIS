package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousMovement3", group = "Autonomous")
public class AutonomousMovement3 extends LinearOpMode {

    private double forwardDistance=0;

    private double backwardDistance=0;

    private double rightAngle=0;

    private double leftAngle=0;

    private double strafeRightDistance=0;

    private double strafeLeftDistance=0;



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
                .strafeRight(strafeLeftDistance)
                .build();

        drive.followTrajectory(strafeLeftTrajectory);

    }
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if(isStopRequested()){
            return;
        }

        forward(25.0);
        turnRight(90.0);
        turnLeft(90.0);
        forward(15.0);
        turnRight(90.0);
        forward(62.0);
        strafeRight(20.0);

    }
}

