
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Core.vvHardware;
import org.firstinspires.ftc.teamcode.Core.vvHardwareRR;
import org.firstinspires.ftc.teamcode.Core.vvRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Auton Tuning class for breaking down the trajectories and rapid testing
 */

@TeleOp(name="vvTuningRR", group="2-TeleOp")
@Disabled
public class vvAutonTuningRR extends LinearOpMode {

    //vvHardware class external pull
    vvHardwareRR robot = new vvHardwareRR(this);

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        robot.init();

        vvRoadRunnerDrive vvdrive = new vvRoadRunnerDrive(hardwareMap);

        /** Starting locations on Centerstage field
         * Red Stage x: -41, y: -65, heading: 90 degrees
         * Blue Stage x: -41, y: 65, heading: 270 degrees
         * Red Back x: 17, y: -65, heading: 90 degrees
         * Blue Back x: 17, y: 65, heading: 270 degrees
         * Stage Door, Arm 89, Pickup -42
         * Stack Arm 17 Pickup -12
        **/
        Pose2d startPose = new Pose2d(-40, -63, Math.toRadians(90));

        vvdrive.setPoseEstimate(startPose);

        //Truss Lap Race
        TrajectorySequence trussRace = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(118)
                .strafeLeft(76)
                .back(116)
                .strafeRight(76)
                .forward(114)
                .strafeLeft(76)
                .back(116)
                .strafeRight(76)
                .forward(114)
                .strafeLeft(76)
                .back(116)
                .strafeRight(76)
                .build();
        // Purple Stage Drop Trajectories - Blue **Note these are flipped in the Backdrop
        TrajectorySequence purpleDropTopBlue = vvdrive.trajectorySequenceBuilder(startPose) //Also Red Back
                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .build();
        TrajectorySequence purpleDropLeftBlue = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .forward(9)
                .waitSeconds(1)
                .back(3)
                .build();
        TrajectorySequence purpleDropRightBlue = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .back(4)
                .waitSeconds(1)
                .build();
        // Purple Stage Drop Trajectories - Red **Note these are flipped in the Backdrop
        TrajectorySequence purpleDropTopRed = vvdrive.trajectorySequenceBuilder(startPose) //Also Blue Back
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.movePickUp(robot.autonPickupIdle - 5, robot.pickUpPwr))
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .waitSeconds(1)
                .build();
        TrajectorySequence purpleDropLeftRed = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .forward(9)
                .waitSeconds(1)
                .back(9)
                .build();
        TrajectorySequence purpleDropRightRed = vvdrive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .turn(Math.toRadians(-50))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.armPos(robot.armIdle + 5, robot.armEPower))
                .forward(6)
                .waitSeconds(1)
                .build();
        // Yellow Stage Drop Trajectories - Red
        TrajectorySequence yellowStageDropRightRed = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armStack, robot.armEPower))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .forward(12)
                .strafeRight(44)
                .UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupStack, robot.pickUpPwr))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(0.9))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.rightWheel.setPower(0))
                .lineToConstantHeading(new Vector2d(24, 3))
                .UNSTABLE_addDisplacementMarkerOffset(-62, () -> robot.armPos(robot.armDoor, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-60,() -> robot.movePickUp(robot.autonPickupDoor,robot.pickUpPwr))
                .UNSTABLE_addDisplacementMarkerOffset(-10, () -> robot.armPos(robot.armLow, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-8,() -> robot.movePickUp(robot.autonPickupLow,robot.pickUpPwr))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42,-44),Math.toRadians(0))
                .forward(8)
                .build();
        TrajectorySequence yellowStageDropTopRed = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armStack, robot.armEPower))
                .turn(Math.toRadians(90))
                .forward(12)
                .strafeRight(44)
                .UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupStack, robot.pickUpPwr))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(0.9))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.rightWheel.setPower(0))
                .lineToConstantHeading(new Vector2d(24, 3))
                .UNSTABLE_addDisplacementMarkerOffset(-62, () -> robot.armPos(robot.armDoor, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-60,() -> robot.movePickUp(robot.autonPickupDoor,robot.pickUpPwr))
                .UNSTABLE_addDisplacementMarkerOffset(-10, () -> robot.armPos(robot.armLow, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-8,() -> robot.movePickUp(robot.autonPickupLow,robot.pickUpPwr))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42,-28),Math.toRadians(0))
                .forward(12)
                .build();

        // Yellow Stage Drop Trajectories - Blue
        TrajectorySequence yellowStageDropTopBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armStack, robot.armEPower))
                .turn(Math.toRadians(-90))
                .forward(12)
                .strafeLeft(43)
                .UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupStack, robot.pickUpPwr))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(0.9))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.rightWheel.setPower(0))
                .lineToConstantHeading(new Vector2d(24, -3))
                .UNSTABLE_addDisplacementMarkerOffset(-62, () -> robot.armPos(robot.armDoor, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-60,() -> robot.movePickUp(robot.autonPickupDoor,robot.pickUpPwr))
                .UNSTABLE_addDisplacementMarkerOffset(-10, () -> robot.armPos(robot.armLow, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-8,() -> robot.movePickUp(robot.autonPickupLow,robot.pickUpPwr))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42,-30),Math.toRadians(0))
                .forward(6)
                .build();
        TrajectorySequence yellowStageDropLeftRed = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(4)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armStack, robot.armEPower))
                .strafeRight(2)
                .forward(20)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(24, 3))
                .UNSTABLE_addDisplacementMarkerOffset(-62, () -> robot.armPos(robot.armDoor, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-60,() -> robot.movePickUp(robot.autonPickupDoor,robot.pickUpPwr))
                .UNSTABLE_addDisplacementMarkerOffset(-10, () -> robot.armPos(robot.armLow, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-8,() -> robot.movePickUp(robot.autonPickupLow,robot.pickUpPwr))
                /*.UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupStack, robot.pickUpPwr))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(0.9))*/
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42,-30),Math.toRadians(0))
                .forward(6)
                .build();
        // Yellow Backdrop Trajectories - Blue
        TrajectorySequence yellowBackDropTopBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .turn(Math.toRadians(90))
                .forward(44)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .forward(2)
                .build();
        TrajectorySequence yellowBackDropRightBlue = vvdrive.trajectorySequenceBuilder(purpleDropRightRed.end())
                .back(6)
                .turn(Math.toRadians(135))
                .forward(42)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(9)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .forward(4)
                .build();
        TrajectorySequence yellowBackDropLeftBlue = vvdrive.trajectorySequenceBuilder(purpleDropLeftRed.end())
                .strafeLeft(10)
                .turn(Math.toRadians(35))
                .forward(44)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr))
                .forward(4)
                .build();
        //Yellow Stage Trajectories - Blue
        TrajectorySequence yellowStageDropLeftBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armStack, robot.armEPower))
                .turn(Math.toRadians(-150))
                .forward(12)
                .strafeLeft(40)
                .UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupStack, robot.pickUpPwr))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(0.9))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.rightWheel.setPower(0))
                .lineToConstantHeading(new Vector2d(24, -3))
                .UNSTABLE_addDisplacementMarkerOffset(-62, () -> robot.armPos(robot.armDoor, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-60,() -> robot.movePickUp(robot.autonPickupDoor,robot.pickUpPwr))
                .UNSTABLE_addDisplacementMarkerOffset(-10, () -> robot.armPos(robot.armLow, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-8,() -> robot.movePickUp(robot.autonPickupLow,robot.pickUpPwr))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42,38),Math.toRadians(0))
                .forward(6)
                .build();
        TrajectorySequence yellowStageDropRightBlue = vvdrive.trajectorySequenceBuilder(purpleDropTopRed.end())
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armStack, robot.armEPower))
                .turn(Math.toRadians(-45))
                .strafeRight(10)
                .forward(12)
                .strafeLeft(40)
                .UNSTABLE_addDisplacementMarkerOffset(-6, () -> robot.movePickUp(robot.autonPickupStack, robot.pickUpPwr))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> robot.rightWheel.setPower(0.9))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.rightWheel.setPower(0))
                .lineToConstantHeading(new Vector2d(24, -3))
                .UNSTABLE_addDisplacementMarkerOffset(-62, () -> robot.armPos(robot.armDoor, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-60,() -> robot.movePickUp(robot.autonPickupDoor,robot.pickUpPwr))
                .UNSTABLE_addDisplacementMarkerOffset(-10, () -> robot.armPos(robot.armLow, robot.armEPower))
                .UNSTABLE_addDisplacementMarkerOffset(-8,() -> robot.movePickUp(robot.autonPickupLow,robot.pickUpPwr))
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42,22),Math.toRadians(0))
                .forward(6)
                .build();
        // Yellow Backdrop Trajectories - Red
        TrajectorySequence yellowBackDropTopRed = vvdrive.trajectorySequenceBuilder(purpleDropTopBlue.end())
                .turn(Math.toRadians(-90))
                .forward(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(4)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .forward(4)
                .build();
        TrajectorySequence yellowBackDropRightRed = vvdrive.trajectorySequenceBuilder(purpleDropRightBlue.end())
                .back(6)
                .turn(Math.toRadians(-45))
                .strafeRight(8)
                .forward(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(11)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .forward(3)
                .build();
        TrajectorySequence yellowBackDropLeftRed = vvdrive.trajectorySequenceBuilder(purpleDropLeftBlue.end())
                .back(6)
                .turn(Math.toRadians(-145))
                .forward(48)
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> robot.armPos(robot.armLow, robot.armEPower))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .forward(4)
                .build();
        // Final Strafe Trajectories - Blue
        TrajectorySequence blueTopEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropTopBlue.end())
                .back(6)
                .strafeLeft(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence blueRightEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropRightBlue.end())
                .back(6)
                .strafeLeft(56)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence blueLeftEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropLeftBlue.end())
                .back(6)
                .strafeLeft(36)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        // Final Strafe Trajectories - Red
        TrajectorySequence redTopEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropTopRed.end())
                .back(6)
                .strafeRight(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence redRightEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropRightRed.end())
                .back(4)
                .strafeRight(36)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        TrajectorySequence redLeftEnd = vvdrive.trajectorySequenceBuilder(yellowBackDropLeftRed.end())
                .back(6)
                .strafeRight(56)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        // Final Strafe Trajectories - Red, moving to the center
        TrajectorySequence redStageTopEnd = vvdrive.trajectorySequenceBuilder(yellowStageDropTopRed.end())
                .back(6)
                .strafeLeft(38)
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.movePickUp(robot.autonPickupIdle, robot.pickUpPwr))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.armPos(robot.armIdle, robot.armEPower))
                .build();
        // initialize all the hardware
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Pose2d poseEstimate = vvdrive.getPoseEstimate();
            vvdrive.update();

            if (gamepad1.dpad_up) { //Stage Top, Blue
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad1.dpad_left) { //Stage Left
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad1.dpad_right) { //Stage Right
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad1.dpad_down) { //Stage Right
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                vvdrive.followTrajectorySequence(trussRace);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if (gamepad1.y) { //Backdrop Top Blue
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopRed);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropTopBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(blueTopEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }

            if (gamepad1.x){ //Backdrop Left Blue
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
                sleep(500);
                vvdrive.followTrajectorySequence(yellowBackDropLeftBlue);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(blueLeftEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if(gamepad1.b){ //Backdrop Right Blue
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropRightBlue);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(blueRightEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if (gamepad2.dpad_up) { //Stage Top, Red
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad2.dpad_left) { //Stage Left
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad2.dpad_right) { //Stage Right
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightRed);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
            }
            if (gamepad2.dpad_down) { //Stage Top, Red to Backdrop
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(250);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(purpleDropTopRed);
                robot.rightWheel.setPower(-0.9);
                sleep(500);
                robot.rightWheel.setPower(0);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(yellowStageDropTopRed);
                robot.leftWheel.setPower(0.9);
                robot.rightWheel.setPower(-0.9);
                sleep(500);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(redStageTopEnd);
            }
            if (gamepad2.left_bumper) { //Stage Top, Red to Backdrop
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(250);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(purpleDropRightRed);
                robot.rightWheel.setPower(-0.9);
                sleep(500);
                robot.rightWheel.setPower(0);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(yellowStageDropRightRed);
                robot.leftWheel.setPower(0.9);
                robot.rightWheel.setPower(-0.9);
                sleep(500);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(redStageTopEnd);
            }
            if (gamepad2.right_bumper) { //Stage Top, Red to Backdrop
                telemetry.addLine("Running...");
                telemetry.update();
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(250);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(purpleDropLeftRed);
                robot.rightWheel.setPower(-0.9);
                sleep(500);
                robot.rightWheel.setPower(0);
                robot.armPos(robot.armStart, robot.armEPower);
                vvdrive.followTrajectorySequence(yellowStageDropLeftRed);
                robot.leftWheel.setPower(0.9);
                robot.rightWheel.setPower(-0.9);
                sleep(500);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(redStageTopEnd);
            }
            if (gamepad2.y) { //Backdrop Top Red
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropTopBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropTopRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redTopEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if (gamepad2.x){ //Backdrop Left Red
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropLeftBlue);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropLeftRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redLeftEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if(gamepad2.b){ //Backdrop Right Red
                robot.movePickUp(5, robot.pickUpPwr);
                sleep(500);
                robot.armPos(robot.armStart, robot.armEPower);
                sleep(500);
                vvdrive.followTrajectorySequence(purpleDropRightBlue);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
                robot.rightWheel.setPower(-0.9);
                sleep(1000);
                robot.rightWheel.setPower(0);
                vvdrive.followTrajectorySequence(yellowBackDropRightRed);
                robot.leftWheel.setPower(0.9);
                sleep(1000);
                robot.leftWheel.setPower(0);
                vvdrive.followTrajectorySequence(redRightEnd);
                telemetry.addData("Parallel Position: ", poseEstimate.getX());
                telemetry.addData("Perpendicular Position: ", poseEstimate.getY());
                telemetry.update();
            }
            if(gamepad1.left_bumper) { //Pickup oscillation testing
                robot.armPos(robot.armLow, robot.armEPower);
                sleep(1000);
                robot.movePickUp(robot.autonPickupLow, robot.pickUpPwr);
            };
        }
    }
}

