package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

//@Autonomous
public class SampleOpMode extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        //MecanumDrive drive = new MecanumDrive(hardwareMap);

        // Wait for the start signal
        waitForStart();

        // Example: Set the robot's pose
        //drive.setPoseEstimate(beginPose);

        // Add your autonomous logic here
    }
}
