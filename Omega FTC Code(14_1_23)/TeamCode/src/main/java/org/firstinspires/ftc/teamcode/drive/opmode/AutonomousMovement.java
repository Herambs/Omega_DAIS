package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK

    double cX = 0;
    double cY = 0;

    public static final double objectWidthInRealWorldUnits = 4;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 600;  // Replace with the focal length of the camera in pixels

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

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



    public void lineToHeading(double x, double y, double heading){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        double x1 = x;
        double y1 = y;
        double heading1 = heading;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory lineToLinearHeading = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(heading1)))
                .build();

        drive.followTrajectory(lineToLinearHeading);

    }

    public void strafeHeading(double x, double y){

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        double x1 = x;
        double y1 = y;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory lineToLinearHeading = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(x1, y1))
                .build();

        drive.followTrajectory(lineToLinearHeading);

    }
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
        backward(39.8);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(5);
        turnRight(94);
        forward(72);
        sleep(200);
        strafeLeft(22);
//        auto.channelMotion(0.5, 1500);
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(2800);
//        auto.channelMotion(-0.25, -1500);


    }

    public void leftDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

//        forward(23);
//        lineToHeading(-23, 0, 93.5);
//        strafeHeading(0,-23);
//        lineToHeading(75,0,0);


        auto.servoZero();
        backward(22);
        turnRight(94);
        forward(3.2);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(3.7);
        turnLeft(94);
        backward(21.5);
        turnRight(94);
        forward(73.5);
        sleep(500);
        strafeLeft(28.5);
        auto.channelMotionEncoder(0.5, 1500);
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);

        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.25, -1500);


    }

    public void rightDrop(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);
        backward(22.5);
        turnLeft(93);
        forward(3);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(3.5);
        turnRight(94);
        backward(21.5);
        turnRight(93.5);
        forward(80);

        sleep(500);
        strafeLeft(18.5);
       auto.channelMotionEncoder(0.5,1500);
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);
//
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.25, -1300);

    }

    public void rightDropShort(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(23.0);
        turnLeft(94.0);
        forward(4);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(4);

        turnRight(183.0);
        forward(25);
        sleep(500);
        strafeRight(4);
//        auto.channelMotion(0.5, 1000);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
//        auto.channelMotion(-0.25, 100);
        auto.channelZero();

    }

    public void rightDropShortRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(23.0);
        turnLeft(94);
        forward(2.5);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(3);
        strafeLeft(21.0);
        sleep(500);
        forward(25);
        strafeRight(27);
        auto.channelMotionEncoder(0.5,1600);
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.25, -1300);
    }

    public void centerDropShort(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(42.0);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(9);
        turnRight(94);
        forward(26);
        sleep(200);
        strafeLeft(28);
        auto.channelMotionEncoder(0.5, 1500);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(2800);
        auto.channelMotionDown(-0.35, 1300);
        auto.channelZero();

    }

    public void centerDropShortRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(36.0);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(8);
        turnLeft(94);
        forward(25);
        sleep(500);
        strafeRight(25);
        auto.channelMotionEncoder(0.5, 1500);
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);

        auto.drop();

        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.35, 1300);

    }

    public void leftDropShort(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(23.0);
        turnRight(94);
        forward(3);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(3);
        strafeRight(21);
        forward(25);
        strafeLeft(28.5);
//        auto.channelMotion(0.5, 1000);
        auto.channelZero();
        auto.servoPlace();
        sleep(3300);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
//        auto.channelMotion(-0.25, 100);
        auto.channelZero();

    }

    public void leftDropShortRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(22.0);
        turnRight(94);
        forward(3);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(3);
        turnLeft(182.5);
        forward(25);
        strafeLeft(7);
        auto.channelMotionEncoder(0.5,1600);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.2);


        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.25, -1300);
        auto.channelZero();

    }

    public void centerDropRed(){
        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);


        auto.servoZero();//To be change if servo does not work in  auto code
        backward(37.5);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(8);
        turnLeft(94);
        forward(80);
        strafeRight(27);
        auto.channelMotionEncoder(0.5,1600);
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
        auto.channelMotionDown(-0.25, -1300);
        auto.channelZero();


    }

    public void rightDropRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(24.0);
        turnLeft(94);
        forward(3.4);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(4.2);
        turnLeft(94);
        forward(25.4);
        turnRight(94);
        forward(78);
        sleep(500);
        strafeRight(26);
        auto.channelMotionEncoder(0.5,1600);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.25);
        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.25, -1000);
        auto.channelZero();

    }

    public void leftDropRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(24.0);
        turnRight(94);
        forward(3.2);
        auto.purpleDrop1(-0.15);
        sleep(550);
        auto.purpleDrop1(0);
        backward(4);
        turnLeft(94);
        backward(25);
        turnLeft(91);
        forward(78);
        sleep(500);
        strafeRight(20);

        auto.channelMotionEncoder(0.5,1600);
        auto.channelZero();
        auto.servoPlace();
        sleep(3000);
        auto.ForwardDistance(0.25);
        sleep(500);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(3000);
        auto.channelMotionDown(-0.25, -1000);
        auto.channelZero();

    }
    @Override
    public void runOpMode() throws InterruptedException {

//        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
//        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        controlHubCam.setPipeline(auto.YellowBlobDetectionPipeline());
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


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
//        centerDrop();
 //       rightDropShort();
//        centerDropShort();
//        leftDropShort();
   //     centerDropRed();
//        rightDropRed();
//        centerDropShortRed();
   //     rightDropShortRed();
//        leftDropShortRed();
            leftDropRed();
    }
}
