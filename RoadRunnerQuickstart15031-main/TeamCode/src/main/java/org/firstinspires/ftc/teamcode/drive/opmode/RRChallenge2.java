package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRChallenge2 extends LinearOpMode {
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory GoToBoxL = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(21,-22)).build();

        Trajectory PushBox = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(90,0)).build();


        Trajectory ToFarStack = drive.trajectoryBuilder(new Pose2d())
                        .lineToConstantHeading(new Vector2d(45, 0)).build();

        Trajectory ToTallBox = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(80, 55)).build();

        Trajectory ToTallBox2 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(30, 0)).build();

        Trajectory ToStarfish = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(0, -45)).build();

        Trajectory ToStarfish2 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-34 , 0)).build();

        Trajectory ToDropStarfish = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(34 , 0)).build();

        Trajectory ToDropStarfish2 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(0 , 45)).build();

        waitForStart();

        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(GoToBoxL);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(PushBox);
        drive.turn(Math.toRadians(-90));
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToFarStack);
        drive.turn(Math.toRadians(-90));
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToTallBox);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToTallBox2);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToStarfish);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToStarfish2);
        drive.turn(Math.toRadians(90));
        drive.turn(Math.toRadians(-90));
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToDropStarfish);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(ToDropStarfish2);


    }
    }
