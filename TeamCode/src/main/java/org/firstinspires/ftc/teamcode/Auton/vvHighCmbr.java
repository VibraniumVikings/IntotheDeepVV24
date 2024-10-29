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

import java.util.List;
import java.util.Objects;

    /*
     * Auton with 2 high chamber (one from wall) and park
     * Start the robot left side on the x tile line against the wall
     *
     */
    @Autonomous(name = "vvHighCmbr", group = "2 - Auton", preselectTeleOp="vvTeleOp")

    public class vvHighCmbr extends LinearOpMode {
        vvHardwareITDRR robot = new vvHardwareITDRR(this);

        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {

            vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

            // We want to start the bot at x: 14, y: -60, heading: 90 degrees
            Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));

            vvdrive.setPoseEstimate(startPose);

            TrajectorySequence fwdHighChmbr = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                    .forward(27)
                    .waitSeconds(0.5)
                    .build();
            TrajectorySequence sample1Pick  = vvdrive.trajectorySequenceBuilder(fwdHighChmbr.end())
                    .back(6)
                    .turn(Math.toRadians(180))
                    .strafeLeft(83)
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        robot.armPos(robot.armWall, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(50,robot.extArmEPower); })
                    .forward(10)
                    .build();
            TrajectorySequence sample1drop = vvdrive.trajectorySequenceBuilder(sample1Pick.end())
                    .strafeRight(60)
                    .turn(Math.toRadians(180))
                    .forward (17)
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(robot.extArmHighCe,robot.extArmEPower );
                    })
                    .build();
            TrajectorySequence sample2Pick = vvdrive.trajectorySequenceBuilder(sample1drop.end()) //Also Blue Back
                    .turn(Math.toRadians(180))
                    .strafeLeft(53)
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        robot.armPos(robot.armWall, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(50,robot.extArmEPower);
                    })
                    .forward(10)
                    .build();
            TrajectorySequence sample2Drop = vvdrive.trajectorySequenceBuilder(sample2Pick.end())
                    .strafeRight(48)
                    .turn(Math.toRadians(180))
                    .forward (12  )
                    .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristWall();
                        robot.extArmPos(robot.extArmHighCe,robot.extArmEPower );
                    })
                    .build();
            TrajectorySequence park = vvdrive.trajectorySequenceBuilder(sample2Drop.end())
                    .strafeLeft(48)
                    .back(24)
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
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristHighCw();
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                    sleep(100);
                    vvdrive.followTrajectorySequence(fwdHighChmbr);
                    sleep(500);
                    robot.armPos(robot.armHighCa-150,robot.armEPower );
                    sleep(100);
                    robot.openClaw();
                    vvdrive.followTrajectorySequence(sample1Pick);
                    sleep(100);
                    robot.closeClaw();
                    robot.armPos(robot.armWall+50,robot.armEPower );
                    vvdrive.followTrajectorySequence(sample1drop);
                    sleep(500);
                    robot.armPos(robot.armHighCa-150,robot.armEPower );
                    sleep(100);
                    robot.openClaw();
                    robot.armPos(0,robot.armEPower);
                    sleep(100);
                    robot.closeClaw();
                    robot.moveWristCarry();
                    robot.extArmPos(0,robot.extArmEPower);
                    robot.rgb.setPosition(0.29);
                    sleep(1000); //cutting due to time
                    //vvdrive.followTrajectorySequence(sample2Pick);
                    //sleep(500);
                    //robot.closeClaw();
                    //robot.armPos(robot.armWall+50,robot.armEPower );
                    //vvdrive.followTrajectorySequence(sample2Drop);
                    //sleep(500);
                    //robot.armPos(robot.armHighCa-100,robot.armEPower );
                    //robot.openClaw();
                    //robot.armPos(0,robot.armEPower);
                    //robot.moveWristCarry();
                    //robot.extArmPos(0,robot.extArmEPower);
                    //robot.rgb.setPosition(0.277);
                    //sleep(1000);
                    telemetry.addData("Parallel Position: ", poseEstimate.getX());
                    telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                    telemetry.update();

                    break;
                }
            }
        }
    }

