package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITDRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

// Auton with 1 high chamber, 2 high baskets and park

/*
 * High Basket Sequence
 * Start the robot with the right side on the X tile line against the wall-
 *
 */
@Autonomous(name = "vvHighBskt", group = "1 - Auton", preselectTeleOp="vvTeleOp")

public class  vvHighBskt extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(-12, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose) //Tile Start Position
                .forward(24)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.extArmPos(0, robot.armEPower))
                .strafeLeft(58)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.led.setPosition(0.7);
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();


                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                })
                .waitSeconds(0)
                .forward(8)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .back(19)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                })
                .turn(Math.toRadians(-80))
                .back(5)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2 = vvdrive.trajectorySequenceBuilder(yellow1Drop.end())
                .forward(8)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                })
                .turn(Math.toRadians(80))
                .strafeLeft(35)
                .forward(18)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2Drop = vvdrive.trajectorySequenceBuilder(yellow2.end())
                .back(17)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.armEPower);
                })
                .turn(Math.toRadians(-60))
                .waitSeconds(0)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow1Drop.end())
                .forward(12)
                .turn(Math.toRadians(70))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(0, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(0, robot.armEPower);
                })
                .forward(29)
                .turn(Math.toRadians(-90))
                .forward(16)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmHighCe, robot.armEPower);
        })
                .waitSeconds(0)
                .build();

        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Pose2d poseEstimate = vvdrive.getPoseEstimate();
                vvdrive.update();

                robot.rgb.setPosition(0.5);
                robot.armPos(robot.armHighCa, robot.armEPower);
                robot.moveWristHighCw();
                robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                sleep(100);
                vvdrive.followTrajectorySequence(fwdHighCmbr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                sleep(250);
                robot.armPos(robot.armHighCa-150,0.4);
                sleep(250);
                robot.openClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(yellow1);
                //robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                robot.closeClaw();
                robot.led.setPosition(0);
                sleep(100);
                vvdrive.followTrajectorySequence(yellow1Drop);
                //robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                sleep(100);
                robot.openClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(yellow2);
                sleep(100);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow2Drop);
                robot.openClaw();
                sleep(500);
                robot.closeClaw();
                robot.moveWristFloor();
                vvdrive.followTrajectorySequence(ascentPark);
                sleep(500);
                robot.rgb.setPosition(0.29);
                //sleep(500); //cutting out early due to time
                //vvdrive.followTrajectorySequence(yellow2Drop);
                //robot.openClaw();
                //sleep(500);
                //vvdrive.followTrajectorySequence(ascentPark);
                //robot.closeClaw();
                //sleep(500);
                //robot.armPos(0,robot.armEPower);
                //robot.moveWristCarry();
                //robot.extArmPos(0,robot.extArmEPower);
                //robot.rgb.setPosition(0.29);
                //sleep(1000);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}
