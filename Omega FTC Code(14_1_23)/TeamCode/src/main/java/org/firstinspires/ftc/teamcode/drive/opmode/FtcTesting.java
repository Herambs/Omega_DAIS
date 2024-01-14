package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "FtcTesting", group = "Autonomous")
public class FtcTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize Road Runner drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        // Define the starting pose
        Pose2d startPose = new Pose2d();

        // Create the first trajectory: Move forward 30 inches
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(30.0)
                .build();

        // Follow the first trajectory
        drive.followTrajectory(forwardTrajectory);

        // Create the second trajectory: Move backward 28 inches
        Trajectory backwardTrajectory = drive.trajectoryBuilder(forwardTrajectory.end())
                .back(28.0)
                .build();

        // Follow the second trajectory
        drive.followTrajectory(backwardTrajectory);

        // Create the third trajectory: Turn right 90 degrees
  //                  Trajectory turnTrajectory = drive.trajectoryBuilder(backwardTrajectory.end())
  //                          .turn(Math.toRadians(90))
  //                          .build();

            // Follow the third trajectory
  //          drive.followTrajectory(turnTrajectory);

        drive.turn(Math.toRadians(-90));

        // Create the fourth trajectory: Move forward 24 inches
        Trajectory forwardAgainTrajectory = drive.trajectoryBuilder(backwardTrajectory.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(68)
                .build();

        // Follow the fourth trajectory
        drive.followTrajectory(forwardAgainTrajectory);

        Trajectory strafeLeftTrajectory= drive.trajectoryBuilder(forwardAgainTrajectory.end())
                .strafeLeft(30)
                .build();

        drive.followTrajectory(strafeLeftTrajectory);



        // Your autonomous code is now complete
    }
}

