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
                .forward(30)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow1 = vvdrive.trajectorySequenceBuilder(fwdHighCmbr.end())
                .resetConstraints()
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.extArmPos(0, robot.armEPower))
                .lineToLinearHeading(new Pose2d(-85,-45,Math.toRadians(90)))
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
                .lineTo(new Vector2d(-119,-60))
                //.lineToLinearHeading(new Pose2d(-106,-65,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower); //
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                })
                //.turn(Math.toRadians(-60))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence yellow2 = vvdrive.trajectorySequenceBuilder(yellow1Drop.end())
                //.turn(Math.toRadians(60))
                //.lineTo(new Vector2d(-105,-45))
                .lineToLinearHeading(new Pose2d(-102,-49,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                })
                .forward(6)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow2Drop = vvdrive.trajectorySequenceBuilder(yellow2.end())
                .lineTo(new Vector2d(-121,-59)) //120
                //.lineToLinearHeading(new Pose2d(-106,-65,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    robot.armPos(robot.armRearBa, robot.armEPower);
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.armEPower);
                })
                //.turn(Math.toRadians(-45))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence yellow3 = vvdrive.trajectorySequenceBuilder(yellow2Drop.end())
                .lineTo(new Vector2d(-98,-60)) //96 47
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.moveWristHighBw();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                    robot.openClaw();
                })
                .lineToLinearHeading(new Pose2d(-95,-23,Math.toRadians(180)))
                .forward(9)
                .waitSeconds(0)
                .build();
        TrajectorySequence yellow3Drop = vvdrive.trajectorySequenceBuilder(yellow3.end())
                .back(5)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-98,-63,Math.toRadians(235)))
                //.lineTo(new Vector2d(-118,-50))
                //.lineToLinearHeading(new Pose2d(-115,-60,Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armHighBa, robot.armEPower);
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristHighBw();
                })
                .waitSeconds(0)
                .build();
        TrajectorySequence ascentPark = vvdrive.trajectorySequenceBuilder(yellow3Drop.end())
                .lineTo(new Vector2d(-100,-30))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmHighCe, robot.armEPower);
                })
                .turn(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-24,-12),Math.toRadians(0))
                .forward(16)
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
                robot.armPos(robot.armHighCa+100, robot.armEPower+0.3);
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
                sleep(100);
                vvdrive.followTrajectorySequence(yellow1Drop);
                //robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                sleep(250);
                robot.openClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(yellow2);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(yellow2Drop);
                sleep(350);
                robot.openClaw();
                sleep(150);
                robot.closeClaw();
                robot.moveWristFloor();
                robot.extArmPos(50, robot.armEPower);
                vvdrive.followTrajectorySequence(yellow3);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(yellow3Drop);
                sleep(100);
                robot.openClaw();
                sleep(250);
                robot.closeClaw();
                robot.moveWristFloor();
                vvdrive.followTrajectorySequence(ascentPark);
                sleep(500);
                robot.led.setPosition(0);
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
