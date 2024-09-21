package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRChallenge1 extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory FRBL = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(110,-45)).build();
        Trajectory BLBR1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-25,25))
                .build();
        Trajectory BLBR2 = drive.trajectoryBuilder(BLBR1.end())
                        .lineTo(new Vector2d(0,45)).build();
        Trajectory BRFL = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-110,-40))
                .build();

        waitForStart();
        drive.setPoseEstimate(new Pose2d());
        spin(drive);
        drive.followTrajectory(FRBL);
        spin(drive);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(BLBR1);
        drive.followTrajectory(BLBR2);
        spin(drive);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectory(BRFL);
        spin(drive);
    }
    public void spin(SampleMecanumDrive drive)
    {
        drive.turn(Math.toRadians(360));
    }

}
