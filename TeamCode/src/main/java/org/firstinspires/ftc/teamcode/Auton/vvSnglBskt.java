package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITDRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.Objects;

/*
 * Single Basket drop with a specimen pick
 * Start the robot on the X tile line against the wall
 *
 */
@Autonomous(name = "vvSnglBskt", group = "3 - Auton", preselectTeleOp="vvTeleOp")

public class  vvSnglBskt extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(-12, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);


        TrajectorySequence sampleDrop = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(12)
                .turn(Math.toRadians(-60))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                })
                .back(12)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(sampleDrop.end())
                .forward(13)
                .turn(Math.toRadians(65))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower))
                .strafeLeft(19)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                    robot.openClaw();
                })
                .forward(13)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .back(18)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                })
                .turn(Math.toRadians(-60))
                .back(10)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2 = vvdrive.trajectorySequenceBuilder(yellow1Drop.end()) //Also Blue Back
                .forward(8)
                .turn(Math.toRadians(60))
                .strafeLeft(18)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                })
                .forward(12)
                .waitSeconds(0)
                .build();
        TrajectorySequence dropYellow2 = vvdrive.trajectorySequenceBuilder(yellow2.end())
                .back(12)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristHighBw();
                })
                .turn(Math.toRadians(-60))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighBe, robot.extArmEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow1Drop.end())
                .forward(12)
                .turn(Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(0, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(0, robot.armEPower);
                })
                .forward(30)
                .turn(Math.toRadians(-90))
                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmHighCe, robot.armEPower);
                })
                .waitSeconds(0)
                .build();

        robot.init();

        String spikeLoc;

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Pose2d poseEstimate = vvdrive.getPoseEstimate();
                vvdrive.update();

                robot.rgb.setPosition(0.5);
                vvdrive.followTrajectorySequence(sampleDrop);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                sleep(250);
                robot.openClaw();
                sleep(250);
                vvdrive.followTrajectorySequence(yellow1);
                sleep(250);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow1Drop);
                sleep(250);
                robot.openClaw();
                sleep(500);
                //vvdrive.followTrajectorySequence(yellow2);
                robot.closeClaw();
                //vvdrive.followTrajectorySequence(dropYellow2);
                //sleep(250);
                //robot.openClaw();
                //sleep(250);
                vvdrive.followTrajectorySequence(ascentPark);
                robot.rgb.setPosition(0.28);
                sleep(500);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}
