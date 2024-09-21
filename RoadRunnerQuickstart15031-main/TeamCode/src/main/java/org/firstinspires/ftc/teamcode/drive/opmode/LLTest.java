package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LLTest extends LinearOpMode {
    @Override
            private Limelight3A limelight;

            @Override
            public void runOpMode() throws InterruptedException
            {
                limelight = hardwareMap.get(Limelight3A.class, "limelight");

                telemetry.setMsTransmissionInterval(11);

                limelight.pipelineSwitch(0);

                /*
                 * Starts polling for data.
                 */
                limelight.start();
        .
        .
    }