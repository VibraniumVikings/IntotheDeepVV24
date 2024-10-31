package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous(name = "vvHighCmbr2", group = "4 - Auton", preselectTeleOp="vvTeleOp")

public class vvHighCmbr2 extends LinearOpMode {
    vvHardwareITDRR robot = new vvHardwareITDRR(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        // We want to start the bot at x: 14, y: -60, heading: 90 degrees
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        TrajectorySequence fwdHighChmbr = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                .forward(25)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence sample1Pick  = vvdrive.trajectorySequenceBuilder(fwdHighChmbr.end())
                .back(6)
                .strafeRight(70)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristFloor();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower); })
                .forward(12)
                .build();
        TrajectorySequence sample1drop = vvdrive.trajectorySequenceBuilder(sample1Pick.end())
                .turn(Math.toRadians(-180))
                .forward (19)
                .build();
       /* TrajectorySequence sample2Drop = vvdrive.trajectorySequenceBuilder(sample1.end())
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
*/
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
                sleep(250);
                robot.armPos(robot.armHighCa-150,robot.armEPower );
                sleep(250);
                robot.openClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(sample1Pick);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                vvdrive.followTrajectorySequence(sample1drop);
                robot.openClaw();
                sleep(500);
                robot.armPos(0,robot.armEPower);
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

