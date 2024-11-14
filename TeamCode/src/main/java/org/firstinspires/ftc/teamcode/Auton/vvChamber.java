package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.vvHardwareITDRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Auton with 1 high chamber, pick and retrieve from obs zone, and park
 * Start the robot left side on the x tile line against the wall
 */
@Autonomous(name = "vvChamber", group = "3 - Auton", preselectTeleOp="vvTeleOp")

public class vvChamber extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        /*  MeepMeep            .forward(31)
                                .back(16)
                                .lineToLinearHeading(new Pose2d(34,-36,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(40,-6,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(48,-60,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(60,-30,Math.toRadians(90)))
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(48,-65,Math.toRadians(-90)))
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(6,-36,Math.toRadians(90)))
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(48,-60,Math.toRadians(90)))
          */

        TrajectorySequence fwdHighChmbr = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                .setConstraints(robot.hcv,robot.hca)
                .forward(30)
                .waitSeconds(0)
                .build();
        TrajectorySequence sample1  = vvdrive.trajectorySequenceBuilder(fwdHighChmbr.end())
                .setConstraints(robot.hspdv,robot.hspda)
                .back(8)
                .lineTo(new Vector2d(66,-36))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.closeClaw();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower); })
                .lineTo(new Vector2d(86,-10))
                .lineTo(new Vector2d(112,-10))
                .lineTo(new Vector2d(130,-50))
                .lineTo(new Vector2d(151,-30))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.moveWristFloor();
                    robot.openClaw();
                    })
                .forward(5)
                .waitSeconds(0)
                .build();
        TrajectorySequence sample2 = vvdrive.trajectorySequenceBuilder(sample1.end())
                .resetConstraints()
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(136,-52,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armWall, robot.armEPower);
                    robot.moveWristWall();
                    robot.extArmPos(robot.extArmLowCe, robot.extArmEPower); })
                .forward (7)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.openClaw();
                })
                .waitSeconds(0)
                .build();
        TrajectorySequence sample1Place = vvdrive.trajectorySequenceBuilder(sample2.end())
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(66,-32,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armHighCa+100, robot.armEPower);
                    robot.moveWristHighCw();
                    robot.extArmPos(robot.extArmHighCe,robot.extArmEPower );
                })
                .forward(5)
                .waitSeconds(0)
                .build();
        TrajectorySequence park = vvdrive.trajectorySequenceBuilder(sample1Place.end())
                .lineToLinearHeading(new Pose2d(48,-52,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.closeClaw();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower); })
                .waitSeconds(0)
                .build();

        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Robot Ready");

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Pose2d poseEstimate = vvdrive.getPoseEstimate();
                vvdrive.update();

                robot.rgb.setPosition(0.5);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.armPos(robot.armHighCa+100, robot.armEPower+0.3);
                robot.moveWristHighCw();
                vvdrive.followTrajectorySequence(fwdHighChmbr);
                sleep(200);
                robot.armPos(robot.armHighCa-250,0.4);
                sleep(350);
                robot.openClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(sample1);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(sample2);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(sample1Place);
                sleep(200);
                robot.armPos(robot.armHighCa-250,0.4);
                sleep(350);
                robot.openClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(park);
                robot.led.setPosition(0);
                robot.rgb.setPosition(0.29);
                sleep(500);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();

                break;
            }
        }
    }
}

