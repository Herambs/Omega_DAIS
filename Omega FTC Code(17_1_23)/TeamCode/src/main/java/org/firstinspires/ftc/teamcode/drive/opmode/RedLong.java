package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedLong", group = "Autonomous")
public class RedLong extends LinearOpMode {

    private double forwardDistance=0;

    double width = 0;

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


    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerYellow = new Scalar(0, 50, 70);
        Scalar upperYellow = new Scalar(60, 255, 255);
        //Hue 0-60 for blue
        //Hue 60-90 for green
        //Hue 100-130 for red


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }



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



    public void centerDropRed(){
        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);


        auto.servoZero();//To be change if servo does not work in  auto code
        backward(44);
        auto.purpleDrop1(-0.35);
        sleep(550);
        auto.purpleDrop1(0);
        sleep(5000);
        backward(6);
        sleep(4000);
        turnLeft(94.5);
        forward(75);
        turnRight(94.5);
        forward(23.8);
        turnLeft(94.5);
        //strafeRight(23.8);
        auto.channelMotionEncoder(0.5,1500);
        auto.servoPlace();
        sleep(1000);
        auto.ForwardDistance(0.2);
        sleep(100);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(1000);
        auto.channelMotionDown(-0.5, 0);


    }

    public void rightDropRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(26.5);
        turnLeft(94.5);
        forward(3.4);
        auto.purpleDrop1(-0.35);
        sleep(550);
        auto.purpleDrop1(0);
        sleep(4000);
        backward(4.2);
        turnRight(94.5);
        backward(21);
        turnLeft(94.5);
        forward(76);
        sleep(100);
        turnRight(94.5);
        forward(29.5);
        turnLeft(94.5);
        //strafeRight(27);
        auto.channelMotionEncoder(0.5,1500);
        auto.servoPlace();
        sleep(1000);
        auto.ForwardDistance(0.25);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(1000);
        auto.channelMotionDown(-0.5, 0);

    }

    public void leftDropRed(){

        SampleMecanumDrive auto= new SampleMecanumDrive(hardwareMap);

        auto.servoZero();
        backward(26.5);
        turnRight(94.5);
        forward(3.2);
        auto.purpleDrop1(-0.35);
        sleep(550);
        auto.purpleDrop1(0);
        backward(4.2);
        sleep(4000);
        turnLeft(94.5);
        backward(22);
        turnLeft(94.5);
        forward(76);
        sleep(100);
        turnRight(94.5);
        forward(17.5);
        turnLeft(94.5);
        //strafeRight(16.5);

        auto.channelMotionEncoder(0.5,1500);
        auto.servoPlace();
        sleep(1000);
        auto.ForwardDistance(0.25);
        auto.drop();
        sleep(1000);
        auto.servoZero();
        sleep(1000);
        auto.channelMotionDown(-0.5, 0);

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
//        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

//        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//        telemetry.addData("Distance in Inch", (getDistance(width)));
//        telemetry.update();
//
//        if(cX >= 50 && cX <= 200){
//            telemetry.addLine("LEFT");
//        } else if (cX >= 400 && cX <= 600) {
//            telemetry.addLine("RIGHT");
//        } else if (cX >= 200 && cX <= 400) {
//            telemetry.addLine("CENTER");
//        }
//
//        // Release resources
//        controlHubCam.stopStreaming();

        if (isStopRequested()) {
            return;
        }

            //centerDropRed();
      //  rightDropRed();
        leftDropRed();
    }
}
