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


// Pedro Auton with 1 high chamber, 3 high baskets and park

/*
 * High Basket Sequence
 * Start the robot with the right side on the X tile line against the wall-
 *
 */
@Autonomous(name = "vvBasketPedro", group = "1 - Auton", preselectTeleOp="vvTeleOp")

public class vvBasketPedro extends OpMode {
    private vvHardwareITDPedro robot;

    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, opmodeTimer;

    private Follower follower;

    private Path fwdHighCmbr;

    private PathChain yellow1, yellow1drop, yellow2, yellow2drop, yellow3, yellow3drop, ascent;

    private int pathState;

    // We want to start the bot at x: 14, y: -60, heading: 90 degrees (these are FIRST coordinates)
    private Pose startPose = new Pose(65, 7, Math.toRadians(90));
    // all sample mark locations
    private Pose highchamber = new Pose(65,33.5);
    private Pose sampleMark1 = new Pose(22.5,27);
    private Pose sampleMark2 = new Pose(19,27);
    private Pose sampleMark3 = new Pose(15,27,Math.toRadians(180));
    private Pose dropposition = new Pose (16,16,Math.toRadians(45));
    private Pose specimenMark1 = new Pose(36+72, -45+72);
    private Pose specimenMark2 = new Pose(24.5+72, -45+72);
    private Pose specimenMark3 = new Pose(36+72, -45+72,Math.toRadians(0));
    private Pose ascentPose  = new Pose(48, 72, Math.toRadians(0));
    //Kraken dimensional offsets
    public double botWidth = 7;
    public doulbe botLength = 7;
    public double botPickup = 11;

    public void buildPaths() {

        //High Chamber travel path
        fwdHighCmbr = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN))); //Tile Start Position
        fwdHighCmbr.setConstantHeadingInterpolation(startPose.getHeading());
        fwdHighCmbr.setPathEndTimeoutConstraint(2);

        //Path from submersible to the first yellow sample, approaches straight on after a sweep
        yellow1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(highchamber.getX(), highchamber.getY(),Point.CARTESIAN), new Point(sampleMark1.getX()+1,sampleMark1.getY()-17, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(sampleMark1.getX()+1,sampleMark1.getY()-17, Point.CARTESIAN), new Point(sampleMark1.getX()+1,sampleMark1.getY()-botPickup, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();

        yellow1drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark1.getX()+1,sampleMark1.getY()-botPickup, Point.CARTESIAN), new Point(dropposition.getX(),dropposition.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),dropposition.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        yellow2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropposition.getX(), dropposition.getY(),Point.CARTESIAN), new Point(sampleMark2.getX(),sampleMark2.getY()-botPickup, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(dropposition.getHeading(),startPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        yellow2drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark2.getX(),sampleMark2.getY()-botPickup, Point.CARTESIAN), new Point(dropposition.getX(),dropposition.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),dropposition.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        yellow3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropposition.getX(), dropposition.getY(),Point.CARTESIAN), new Point(sampleMark3.getX()+14,sampleMark3.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(dropposition.getHeading(),sampleMark3.getHeading(),0.5)
                .addPath(new BezierLine(new Point(sampleMark3.getX()+14,sampleMark3.getY(), Point.CARTESIAN), new Point(sampleMark3.getX()+botPickup,sampleMark3.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(sampleMark3.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();

        yellow3drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark3.getX()-botPickup,sampleMark3.getY(), Point.CARTESIAN), new Point(dropposition.getX(),dropposition.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(sampleMark3.getHeading(),dropposition.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        ascent = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropposition.getX(), dropposition.getY(),Point.CARTESIAN), new Point(ascentPose.getX(),ascentPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(dropposition.getHeading(),ascentPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();
    }

    public void autonPathUpdate() {
        switch (pathState) {
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
            case 11: // Yellow1
                if (pathTimer.getElapsedTime() > 2000) {
                    robot.armPos(robot.armHighCa - 250, 0.4);
                    try {
                        Thread.sleep(350);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    robot.openClaw();
                    //robot.extArmPos(50, robot.extArmEPower);
                    follower.followPath(yellow1);

                    setPathState(12);

                    break;

                }
            case 12: //Yellow1pick
                if (pathTimer.getElapsedTime() > 250) {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                    try {
                        Thread.sleep(350);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    robot.closeClaw();
                    setPathState(13);

                    break;
                }
            case 13: //Yellow1drop
                if (pathTimer.getElapsedTime() > 500) {
                    robot.armPos(robot.armRearBa, robot.armEPower - 0.15); //
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristHighBw();

                    follower.followPath(yellow1drop);
                    
                    if(!follower.isBusy()){
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        robot.openClaw();
                    }
                    
                    setPathState(14);

                    break;
                }
            case 14: //Yellow2pick
                if (pathTimer.getElapsedTime() > 250) {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);

                    follower.followPath(yellow2);
                    
                    if(!follower.isBusy()){
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        robot.closeClaw();
                    }
                    setPathState(15);

                    break;
                }
            case 15: //Yellow2drop
                if (pathTimer.getElapsedTime() > 500) {
                    robot.armPos(robot.armRearBa, robot.armEPower - 0.15); //
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristHighBw();

                    follower.followPath(yellow2drop);
                    
                    if(!follower.isBusy()){
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        robot.openClaw();
                    }
                    
                    setPathState(16);

                    break;
                }
            case 16: //Yellow3pick
                if (pathTimer.getElapsedTime() > 250) {
                    robot.moveWristFloor();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);

                    follower.followPath(yellow3);
                    
                    if(!follower.isBusy()){
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        robot.closeClaw();
                    }
                    
                    setPathState(17);

                    break;
                }
            case 17: //Yellow3drop
                if (pathTimer.getElapsedTime() > 500) {
                    robot.armPos(robot.armRearBa, robot.armEPower - 0.15); //
                    robot.moveWristLowCW();
                    robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                    robot.moveWristHighBw();

                    follower.followPath(yellow3drop);
                    
                    if(!follower.isBusy()){
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        robot.openClaw();
                    }
                    
                    setPathState(18);

                    break;
                }
            case 18: //parking
                if (pathTimer.getElapsedTime() > 500) {

                    robot.closeClaw();
                    robot.wristCarry();
                    robot.extArmPos(0, robot.extArmEPower);
                    robot.armPos(robot.armAscent, robot.extArmEPower);

                    follower.followPath(ascent);
                    
                    setPathState(100);
                    
                break;
                 }
            case 100:
                if (!follower.isBusy()) {
                    //setPathState(-1);
                }
                break;

            default:
                requestOpModeStop();
                break;

        }
    }

                /*follower.followPath(yellow1);
                //robot.extArmPos(robot.extArmFLoorPick, robot.extArmEPower);
                robot.closeClaw();
                sleep(100);
                follower.followPath(yellow1Drop);
                //robot.extArmPos(robot.extArmHighBe, robot.extArmEPower);
                sleep(250);
                robot.openClaw();
                sleep(100);
                follower.followPath(yellow2);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                follower.followPath(yellow2Drop);
                sleep(350);
                robot.openClaw();
                sleep(150);
                robot.closeClaw();
                robot.moveWristFloor();
                robot.extArmPos(50, robot.armEPower);
                follower.followPath(yellow3);
                sleep(100);
                robot.closeClaw();
                sleep(100);
                follower.followPath(yellow3Drop);
                sleep(100);
                robot.openClaw();
                sleep(250);
                robot.closeClaw();
                robot.moveWristFloor();
                follower.followPath(ascentPark);
                sleep(500);
                robot.led.setPosition(0);
                robot.rgb.setPosition(0.29);
                sleep(500); //cutting out early due to time
                vvdrive.followTrajectorySequence(yellow2Drop);
                robot.openClaw();
                sleep(500);
                vvdrive.followTrajectorySequence(ascentPark);
                robot.closeClaw();
                sleep(500);
                robot.armPos(0,robot.armEPower);
                robot.moveWristCarry();
                robot.extArmPos(0,robot.extArmEPower);
                robot.rgb.setPosition(0.29);
                sleep(1000);
                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.update();
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                break;*/
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
