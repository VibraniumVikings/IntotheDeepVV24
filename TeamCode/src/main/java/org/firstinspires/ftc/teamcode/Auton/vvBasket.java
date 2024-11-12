package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.vvHardwareITDRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;


// Auton with 1 high chamber, 2 high baskets and park

/*
 * High Basket Sequence
 * Start the robot with the right side on the X tile line against the wall-
 *
 */
@Autonomous(name = "vvBasket", group = "3 - Auton", preselectTeleOp="vvTeleOp")

public class vvBasket extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(-12, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence fwdHighCmbr = vvdrive.trajectorySequenceBuilder(startPose)//Tile Start Position
                .setConstraints(robot.hcv,robot.hca)
                .forward(31)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .resetConstraints()
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.extArmPos(0, robot.armEPower))
                .lineToLinearHeading(new Pose2d(-89,-43,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.led.setPosition(0.7);
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                })
                .waitSeconds(0)
                .forward(5)
                .build();
        TrajectorySequence yellow1Drop = vvdrive.trajectorySequenceBuilder(yellow1.end())
                .lineToLinearHeading(new Pose2d(-96,-78,Math.toRadians(30)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower); //
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow2 = vvdrive.trajectorySequenceBuilder(yellow1Drop.end())
                .lineToLinearHeading(new Pose2d(-116,-43,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                })
                .forward(5)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2Drop = vvdrive.trajectorySequenceBuilder(yellow2.end())
                .lineToLinearHeading(new Pose2d(-96,-78,Math.toRadians(30)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.armEPower);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence yellow3 = vvdrive.trajectorySequenceBuilder(yellow2Drop.end())
                .lineToLinearHeading(new Pose2d(-90,-24,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                    robot.openClaw();
                })
                .forward(5)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow3Drop = vvdrive.trajectorySequenceBuilder(yellow3.end())
                .lineToLinearHeading(new Pose2d(-96,-78,Math.toRadians(30)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.armEPower);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow3Drop.end())
                .splineToLinearHeading(new Pose2d(-24,-12),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.moveWristCarry();
                    robot.armPos(0, robot.armEPower);
                    robot.extArmPos(0, robot.armEPower);
                })
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
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.armPos(robot.armHighCa+100, robot.armEPower);
                robot.moveWristHighCw();
                vvdrive.followTrajectorySequence(fwdHighCmbr);
                sleep(200);
                robot.armPos(robot.armHighCa-250,0.4);
                sleep(350);
                robot.openClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(yellow1);
                //robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                robot.closeClaw();
                robot.led.setPosition(0);
                sleep(100);
                vvdrive.followTrajectorySequence(yellow1Drop);
                //robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                sleep(250);
                robot.openClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(yellow2);
                sleep(100);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow2Drop);
                sleep(500);
                robot.openClaw();
                sleep(500);
                robot.closeClaw();
                robot.moveWristFloor();
                vvdrive.followTrajectorySequence(yellow3);
                sleep(100);
                robot.closeClaw();
                vvdrive.followTrajectorySequence(yellow3Drop);
                sleep(500);
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
