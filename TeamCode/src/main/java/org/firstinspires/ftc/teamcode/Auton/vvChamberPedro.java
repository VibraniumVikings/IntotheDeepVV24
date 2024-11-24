package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.vvHardwareITDPedro;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/*
 * Auton with 1 high chamber, move two alliance samples to obs zone, pick and retrieve from obs zone, and park
 * Start the robot left side on the x tile line against the wall
 */
@Autonomous(name = "vvChamberPedro", group = "2 - Auton", preselectTeleOp="vvTeleOp")

public class vvChamberPedro extends OpMode {
    private vvHardwareITDPedro robot;

    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, opmodeTimer;

    private Follower follower;

    private Path fwdHighCmbr;

    private PathChain sample1, sample1drop,sample2, sample2Drop,sample1pick, sample1Place, sample2Pick, sample2Place, obsZone;

    private int pathState;

    // We want to start the bot at x: 14, y: -60, heading: 90 degrees
    private Pose startPose = new Pose(7+72, -65+72, Math.toRadians(90));
    // all sample mark locations
    private Pose DropPosition = new Pose (-56+72,-56+72);
    private Pose sampleMark1 = new Pose(-49.5+72,-45+72);
    private Pose sampleMark2 = new Pose(-12+72,-45+72);
    private Pose sampleMark3 = new Pose(36+72,-45+72);
    private Pose specimenMark1 = new Pose(36+72, -45+72);
    private Pose specimenMark2 = new Pose(24.5+72, -45+72);
    private Pose specimenMark3 = new Pose(36+72, -45+72);
    private Pose observationZone= new Pose(-11+72,-56+72);

    public void buildPaths() {

        fwdHighCmbr = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), new Point(7 + 72, -38.5 + 72, Point.CARTESIAN))); //Tile Start Position
        fwdHighCmbr.setConstantHeadingInterpolation(startPose.getHeading());
        fwdHighCmbr.setPathEndTimeoutConstraint(3);

        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(7 + 72, -38.5 + 72), new Point(specimenMark1.getX()+1, specimenMark1.getY()-17, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(sampleMark1.getX()+1,sampleMark1.getY()-17, Point.CARTESIAN), new Point(sampleMark1.getX()+1,sampleMark1.getY()-6, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        sample1drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenMark1.getX(),specimenMark1.getY(), Point.CARTESIAN), new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(specimenMark1.getHeading()+180)
                .setPathEndTimeoutConstraint(0)
                .build();

        sample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN), new Point(specimenMark2.getX(),specimenMark2.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(observationZone.getHeading()+180)
                .setPathEndTimeoutConstraint(0)
                .build();

        sample1pick = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenMark2.getX(),specimenMark2.getY(), Point.CARTESIAN), new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(specimenMark2.getHeading()+180)
                .setPathEndTimeoutConstraint(0)
                .build();

        sample1Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN), new Point(7 + 72,-36 + 72, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(observationZone.getHeading()+180)
                .setPathEndTimeoutConstraint(0)
                .build();

        sample2Pick = follower.pathBuilder()
                .addPath(new BezierLine(new Point(7 + 72,-36 + 72, Point.CARTESIAN), new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(observationZone.getHeading()+180)
                .setPathEndTimeoutConstraint(0)
                .build();

        sample2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN), new Point(7 + 72,-38 + 72, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(observationZone.getHeading()+180)
                .setPathEndTimeoutConstraint(0)
                .build();
    }



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
        
        TrajectorySequence sample1  = vvdrive.trajectorySequenceBuilder(fwdHighChmbr.end())
                //.setConstraints(robot.hspdv,robot.hspda)
                .back(8)
                .lineTo(new Vector2d(63,-36))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.closeClaw();
                    robot.extArmPos(robot.extArmFLoorPick+50, robot.extArmEPower);
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristCarry();
                     })
                .lineTo(new Vector2d(85,-10))
                .lineTo(new Vector2d(102,-10))
                .lineTo(new Vector2d(117,-55))
                .lineTo(new Vector2d(140,-33))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.moveWristFloor();
                    robot.openClaw();
                    })
                .forward(4)
                .waitSeconds(0.25)
                .build();
        TrajectorySequence sample2 = vvdrive.trajectorySequenceBuilder(sample1.end())
                //.resetConstraints()
                .lineToLinearHeading(new Pose2d(135,-47,Math.toRadians(225)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.armPos(robot.armWall, robot.armEPower);
                    robot.moveWristWall();
                    robot.extArmPos(robot.extArmLowCe, robot.extArmEPower); })
                .turn(Math.toRadians(45))
                .forward (15.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.65, () -> {
                    robot.openClaw();
                })
                .waitSeconds(0)
                .build();
        TrajectorySequence sample1Place = vvdrive.trajectorySequenceBuilder(sample2.end())
                .back(6)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(70,-45,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristHighCw();
                    robot.extArmPos(robot.extArmHighCe,robot.extArmEPower );
                })
                .waitSeconds(0)
                .build();
        TrajectorySequence park = vvdrive.trajectorySequenceBuilder(sample1Place.end())
                //.lineToLinearHeading(new Pose2d(120,-52,Math.toRadians(0)))
                .lineTo(new Vector2d(120,-55))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.closeClaw();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.moveWristCarry();
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower); })
                .waitSeconds(0)
                .build();
        */
        public void autonPathUpdate() {
        switch (pathState){
            case 9: //move arm to position
                robot.rgb.setPosition(0.5);
                robot.armPos(robot.armHighCa, robot.armEPower);
                //robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                robot.moveWristHighCw();

                setPathState(10);
                break;

            case 10: //high chamber specimen placement
                if (pathTimer.getElapsedTime() > 400) {
                    follower.followPath(fwdHighCmbr);

                    setPathState(11);

                    break;
                }
            case 11:
                if (pathTimer.getElapsedTime() > 2000) {
                    robot.armPos(robot.armHighCa-250,0.4);
                    try {
                        Thread.sleep(350);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    robot.openClaw();
                robot.extArmPos(50, robot.extArmEPower);
                follower.followPath(sample1);

                setPathState(100);

                break;

            }
            case 12:
                if (!follower.isBusy()) {
                    //setPathState(-1);
                }
                break;

            default:
                requestOpModeStop();
                break;

        }
    }
    @Override
    public void loop() {

        follower.update();

        autonPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init() {
        //foldUp = new SingleRunAction(()-> {
        //    if (Integer.parseInt(String.valueOf(pathState).substring(0,1)) < 4) setPathState(50);
        //});

        robot = new vvHardwareITDPedro(true);
        robot.hardwareMap = hardwareMap;
        //robot.telemetry = telemetry;

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        robot.init();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

    }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonPathUpdate();
    }
    @Override
    public void start() {

                buildPaths();

                resetRuntime();

                setPathState(9);

            }
    @Override
    public void stop() {
    }
}
/*
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
                sleep(50);
                robot.closeClaw();
                sleep(50);
                vvdrive.followTrajectorySequence(sample2);
                sleep(350);
                robot.closeClaw();
                sleep(50);
                vvdrive.followTrajectorySequence(sample1Place);
                sleep(100);
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
}*/

