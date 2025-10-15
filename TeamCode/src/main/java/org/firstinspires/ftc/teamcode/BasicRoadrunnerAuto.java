package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BasicRoadrunnerAuto", group = "Autonomous")
public class BasicRoadrunnerAuto extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Roadrunner mecanum drive
        drive = new MecanumDrive(hardwareMap, hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu"));

        // Wait for start command
        waitForStart();

        if (isStopRequested()) return;

        // Define starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Build a simple trajectory forward 30 inches
        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        // Follow the trajectory
        drive.followTrajectory(forwardTrajectory);

        // Wait until the movement finishes
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            idle();
        }

        // Build a turn by 90 degrees (pi/2 radians)
        drive.turn(Math.toRadians(90));

        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            idle();
        }

        // Build a strafe left 20 inches trajectory
        Trajectory strafeLeftTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(20)
                .build();

        drive.followTrajectory(strafeLeftTrajectory);

        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            idle();
        }
    }
}
