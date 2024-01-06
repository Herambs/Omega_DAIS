package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private Servo boxLeft, boxRight;
    private Servo InnerGrab;

    private DcMotorEx leftslider, rightslider;

    DistanceSensor dsensor;

//    public void channelMotion(double power){
//        double set=power;
//        leftslider.setPower(set);
//        rightslider.setPower(set);
//    }
//
//
//
//    public void channelZero(){
//        leftslider.setPower(0.1);
//        rightslider.setPower(0.1);
//    }
//
//    public void servoPlace(){
//        boxLeft.setPosition(0.55);//0.7 for longer distance auto
//        boxRight.setPosition(0.45);//0.3 for longer distance auto
//    }
//    public void drop(){
//        InnerGrab.setPosition(0.32);
//    }

//    public  void servoZero(){
//        boxLeft.setPosition(0.02);
//        boxRight.setPosition(0.98);
//        InnerGrab.setPosition(0.75);
//    }


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
        backward(46.0);  // Center Drop Forward = 48
        turnRight(184);
        //drop purple pixel
        backward(3.0);
        turnLeft(92);
        forward(74.5);

//        auto.StrafeLeftDistance(0.6);
//        strafeLeft(7.0);
        auto.channelMotion(0.7);
        sleep(1000);
        auto.channelZero();
        sleep(1000);
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
        backward(46);
        turnRight(92);
        //purple drop
        strafeRight(20);
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
        backward(46);
        turnLeft(92);
        turnRight(184);
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

    public void rightDropShortRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(25.0);
        turnLeft(92.5);
        strafeLeft(21.0);
        sleep(500);
        forward(25);
        strafeRight(26.5);
        auto.channelMotion(0.5);
        sleep(1500);//1600 for longer distance auto
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.35);
        sleep(2000);
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

    public void centerDropShortRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(46.0);
        turnLeft(93.5);
        forward(25);
        sleep(500);
        strafeRight(20);
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
        auto.channelMotion(-0.35);
        sleep(2000);
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

    public void leftDropShortRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(25.0);
        turnRight(93.5);
        turnLeft(182.5);
        forward(25);
        strafeLeft(7);
        auto.channelMotion(0.5);
        sleep(1500);//1600 for longer distance auto
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.35);
        sleep(2000);
        auto.channelZero();

    }

    public void centerDropRed(){
        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);





        auto.servoZero();//To be change if servo does not work in  auto code
        backward(46);
//        auto.purpleDrop(0.8);
//        sleep(100);
//        auto.outTakeStop();
//        backward(4);
        turnLeft(92.5);
        forward(74.5);
        strafeRight(21.5);
        auto.channelMotion(0.7);

        sleep(1000);
        auto.channelZero();
        sleep(1000);
        auto.servoPlace();
        sleep(2000);
        auto.ForwardDistance(0.2);
        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotion(-0.35);
        sleep(2000);
        auto.channelZero();


    }

    public void rightDropRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(25.0);
        turnLeft(93.5);
//        turnLeft(180.0);
        strafeLeft(21.0);
        forward(73.5);
        sleep(500);
//        auto.StrafeRightDistance(0.6);
        strafeRight(25);
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
        auto.channelMotion(-0.35);
        sleep(2000);
        auto.channelZero();

    }

    public void leftDropRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(25.0);
        turnRight(93.5);
        turnLeft(182);
        strafeLeft(21.0);
        forward(73.5);
        sleep(500);
//        auto.StrafeRightDistance(0.6);
        strafeRight(14);
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
        auto.channelMotion(-0.35);
        sleep(2000);
        auto.channelZero();

    }
    @Override
    public void runOpMode() throws InterruptedException {


//        boxLeft= hardwareMap.get(Servo.class,"boxLeft");
//        boxRight= hardwareMap.get(Servo.class,"boxRight");
//        InnerGrab= hardwareMap.get(Servo.class,"innerGrab");
//
//        leftslider=hardwareMap.get(DcMotorEx.class,"Left_slider");
//        rightslider=hardwareMap.get(DcMotorEx.class,"Right_slider");
//        dsensor = hardwareMap.get(DistanceSensor.class,"dropping_distance");


        waitForStart();

        if(isStopRequested()){
            return;
        }

//        centerDrop();  // Spike Location center
//        leftDrop();
//        rightDrop();
        centerDrop();
//        rightDropShort();
//        centerDropShort();
//        leftDropShort();
//        centerDropRed();
//        rightDropRed();
//        leftDropRed();
//        centerDropShortRed();
//        rightDropShortRed();
     //   leftDropShortRed();
    }
}
