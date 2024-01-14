package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Position1Path1", group = "Autonomous")
public class Position1Path1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 25 inches
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(25.0)
                .build();

        // Create the second trajectory: Move forward 15 inches
        Trajectory forwardTrajectory1 = drive.trajectoryBuilder(forwardTrajectory.end())
                .forward(15.0)
                .build();

        // Create the second trajectory: Move forward 62 inches
        Trajectory forwardTrajectory2 = drive.trajectoryBuilder(forwardTrajectory1.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(62.0)
                .build();

        // Create the strafe left Trajectory: strafe 20 inches
        Trajectory strafeRightTrajectory= drive.trajectoryBuilder(forwardTrajectory2.end())
                .strafeRight(20.0)
                .build();

        // Create the third forward trajectory: Move forward 12 inches
        Trajectory forwardTrajectory3 = drive.trajectoryBuilder(strafeRightTrajectory.end())
                .forward(12.0)
                .build();

        drive.followTrajectory(forwardTrajectory);
        drive.followTrajectory(forwardTrajectory1);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(forwardTrajectory2);
        drive.followTrajectory(strafeRightTrajectory);
        drive.followTrajectory(forwardTrajectory3);



    }
}
