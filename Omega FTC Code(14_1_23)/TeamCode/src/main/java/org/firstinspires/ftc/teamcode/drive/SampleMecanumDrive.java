package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
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
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(6, 0, 0);

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK

    public static double LATERAL_MULTIPLIER = 1.56;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    double cX = 0;
    double cY = 0;

    public static final double objectWidthInRealWorldUnits = 4;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 600;  // Replace with the focal length of the camera in pixels

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution


    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront,leftslider, rightslider, out_take;

    DistanceSensor dsensor;

    Telemetry telemetry =new Telemetry() {
        @Override
        public Item addData(String caption, String format, Object... args) {
            return null;
        }

        @Override
        public Item addData(String caption, Object value) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return null;
        }

        @Override
        public boolean removeItem(Item item) {
            return false;
        }

        @Override
        public void clear() {

        }

        @Override
        public void clearAll() {

        }

        @Override
        public Object addAction(Runnable action) {
            return null;
        }

        @Override
        public boolean removeAction(Object token) {
            return false;
        }

        @Override
        public void speak(String text) {

        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {

        }

        @Override
        public boolean update() {
            return false;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            return false;
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {

        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {

        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return null;
        }
    };

    private Servo boxLeft, boxRight;
    private Servo InnerGrab;

    double width = 0;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
//        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        leftslider=hardwareMap.get(DcMotorEx.class,"Left_slider");
        rightslider=hardwareMap.get(DcMotorEx.class,"Right_slider");
        out_take = hardwareMap.get(DcMotorEx.class,"Front_intake");
        dsensor = hardwareMap.get(DistanceSensor.class,"dropping_distance");

        boxLeft= hardwareMap.get(Servo.class,"boxLeft");
        boxRight= hardwareMap.get(Servo.class,"boxRight");
        InnerGrab= hardwareMap.get(Servo.class,"innerGrab");



//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
//        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);




        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
//        rightRear.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        leftRear.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        leftslider.setDirection(DcMotorEx.Direction.REVERSE); // add if needed
        out_take.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
         setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
//        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
//        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        return 0.0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void channelMotionEncoder(double power, int position){
        int pos = position;
        double pwr=power;

//        telemetry.addData("Right pos = %d ", rightslider.getCurrentPosition());
//        telemetry.addData("Left pos = %d ", leftslider.getCurrentPosition());
//        telemetry.addLine("Hello");
//        telemetry.update();
        leftslider.setTargetPosition(pos);
        rightslider.setTargetPosition(pos);
        leftslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftslider.getCurrentPosition() <= leftslider.getTargetPosition()) && (rightslider.getCurrentPosition() <= rightslider.getTargetPosition())) {
            leftslider.setPower(pwr);
            rightslider.setPower(pwr);
        }

        leftslider.setPower(0);
        rightslider.setPower(0);


    }

    public void channelMotionTimer(double power){
//        int pos = position;
        double pwr=power;

        //leftslider.setTargetPosition(pos);
        //rightslider.setTargetPosition(pos);
        //leftslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftslider.setPower(pwr);
            rightslider.setPower(pwr);


    }


    public void channelMotionDown(double power, int position){
        int pos = position;
        double pwr=power;

        leftslider.setTargetPosition(pos);
        rightslider.setTargetPosition(pos);
        leftslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftslider.getCurrentPosition() >= leftslider.getTargetPosition()) && (rightslider.getCurrentPosition() >= rightslider.getTargetPosition())) {

            leftslider.setPower(pwr);
            rightslider.setPower(pwr);

        }

        leftslider.setPower(0);
        rightslider.setPower(0);


    }

    public void ForwardDistance(double power){
        double D = dsensor.getDistance(DistanceUnit.CM);
        double set=power;
        while(D >= 17) { //D = 21.5 for longer path
            D = dsensor.getDistance(DistanceUnit.CM);
            rightFront.setPower(set);
            rightRear.setPower(set);
            leftRear.setPower(set);
            leftFront.setPower(set);

//            telemetry.addData("Dsensor value=%f",D);
//            telemetry.update();

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);
    }

    public void StrafeLeftDistance(double power){
        double D = dsensor.getDistance(DistanceUnit.CM);
        double set=power;
        while(D >= 23) {
            D = dsensor.getDistance(DistanceUnit.CM);
            rightFront.setPower(set);
            rightRear.setPower(-set);
            leftRear.setPower(set);
            leftFront.setPower(-set);

//            telemetry.addData("Dsensor value=%f",D);
//            telemetry.update();

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);
    }

    public void StrafeRightDistance(double power){
        double D = dsensor.getDistance(DistanceUnit.CM);
        double set=power;
        while(D >= 23) {
            D = dsensor.getDistance(DistanceUnit.CM);
            rightFront.setPower(-set);
            rightRear.setPower(set);
            leftRear.setPower(-set);
            leftFront.setPower(set);

//            telemetry.addData("Dsensor value=%f",D);
//            telemetry.update();

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);
    }

    public void channelZero(){
        leftslider.setPower(0.1);
        rightslider.setPower(0.1);
    }

    public void purpleDrop(double power, int position){
        out_take.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out_take.setTargetPositionTolerance(10);
        double pwr = power;
        int target_position = position;
        out_take.setTargetPosition(target_position);
        out_take.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(out_take.isBusy()) {
            out_take.setPower(-pwr);
        }
        out_take.setPower(0);
    }
    public void purpleDrop1(double power){
        double pwr = power;
        out_take.setPower(pwr);
    }

    public void outTakeStop(){
        out_take.setPower(0);
    }

    public void servoPlace(){
        boxLeft.setPosition(0.55);//0.7 for longer distance auto
        boxRight.setPosition(0.45);//0.3 for longer distance auto
    }
    public void drop(){
        InnerGrab.setPosition(0.32);
    }
    public  void servoZero(){
        boxLeft.setPosition(0);
        boxRight.setPosition(1);
        InnerGrab.setPosition(0.75);
    }

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



}
