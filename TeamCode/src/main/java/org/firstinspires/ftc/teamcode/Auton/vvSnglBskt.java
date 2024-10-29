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

        TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose) //Tile Start Position
                .forward(27)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.extArmPos(0, robot.armEPower))
                .strafeLeft(63)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                })
                .waitSeconds(1)
                .forward(8)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .turn(Math.toRadians(150))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armHighBa, robot.armEPower);
                    robot.moveWristHighBw();
                })
                .forward(24)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmHighBe, robot.extArmEPower))
                .waitSeconds(1)
                .build();
        TrajectorySequence pickSpecimen = vvdrive.trajectorySequenceBuilder(yellow1Drop.end()) //Also Blue Back
                .back(32)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(0, robot.extArmEPower))
                .turn(Math.toRadians(135))
                .forward(80)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armWall, robot.armEPower);
                    robot.moveWristWall();
                })
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmLowCe, robot.extArmEPower))
                .waitSeconds(0)
                .build();
        TrajectorySequence observPark = vvdrive.trajectorySequenceBuilder(pickSpecimen.end())
                .splineToConstantHeading(new Vector2d(48,-51),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.floorArm, robot.armEPower))
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
                robot.armPos(robot.armHighCa, robot.armEPower);
                robot.moveWristHighCw();
                robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                vvdrive.followTrajectorySequence(fwdHighCmbr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.armPos(robot.armHighCa-150,robot.armEPower);
                robot.openClaw();
                vvdrive.followTrajectorySequence(yellow1);
                robot.armPos(robot.extArmFLoorPick,robot.armEPower);
                sleep(500);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow1Drop);
                robot.openClaw();
                vvdrive.followTrajectorySequence(pickSpecimen);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(observPark);
                robot.openClaw();
                robot.armPos(0,robot.armEPower);
                robot.moveWristCarry();
                robot.extArmPos(0,robot.extArmEPower);
                robot.rgb.setPosition(0.28);
                sleep(1000);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}
