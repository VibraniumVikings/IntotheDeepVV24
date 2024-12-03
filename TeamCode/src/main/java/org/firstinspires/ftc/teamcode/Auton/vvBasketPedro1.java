package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "vvBasketPedro1", group = "1 - Auton", preselectTeleOp="vvTeleOp")
@Disabled
public class vvBasketPedro1 extends OpMode {
    private vvHardwareITDPedro robot;

    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, opmodeTimer;

    private Follower follower;

    private Path fwdHighCmbr;

    private PathChain yellow1, yellow1drop, yellow2, yellow2drop, yellow3, yellow3drop, ascent;

    private int pathState;

    // We want to start the bot at x: 65, y: 7, heading: 90 degrees (these are FIRST coordinates)
    private Pose startPose = new Pose(65, 7, Math.toRadians(90));
    // all sample mark locations
    private Pose highchamber = new Pose(65,33.5);
    private Pose sampleMark1 = new Pose(22.5,43);
    private Pose sampleMark2 = new Pose(12.5,43);
    private Pose sampleMark3 = new Pose(2.5,43,Math.toRadians(180));
    private Pose dropposition = new Pose (16,16,Math.toRadians(45));
    private Pose specimenMark1 = new Pose(121.5, 43);
    private Pose specimenMark2 = new Pose(131.5, 43);
    private Pose specimenMark3 = new Pose(141.5, 43,Math.toRadians(0));
    private Pose ascentPose  = new Pose(72, 72, Math.toRadians(0));
    //Kraken dimensional offsets
    public double botWidth = 7;
    public double botLength = 7;
    public double botPickup = 11;

    public boolean isHalfwayThere() {
        return follower.getCurrentTValue() > 0.5;
    }
    public boolean atPathEnd() {
        return follower.getCurrentTValue() > 0.99;
    }
    public boolean isAtEndOfPathAndNotMoving() { //Robot seems unstable with control, along with isBusy
        return atPathEnd() && follower.getVelocityMagnitude() < 0.1;
    }
    public boolean armSetUp() {
        return robot.arm.getCurrentPosition()>robot.arm.getTargetPosition()-10;
    }
    public boolean armSetDown() {
        return robot.arm.getCurrentPosition()>robot.arm.getTargetPosition()+5;
    }

    public void buildPaths() {

        //High Chamber travel path
        fwdHighCmbr = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN))); //Tile Start Position
        fwdHighCmbr.setConstantHeadingInterpolation(startPose.getHeading());
        fwdHighCmbr.setPathEndTimeoutConstraint(2);

        //Path from submersible to the first yellow sample, approaches straight on after a sweep
        yellow1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(highchamber.getX(), highchamber.getY(),Point.CARTESIAN), new Point(47,16, Point.CARTESIAN), new Point(sampleMark1.getX(),sampleMark1.getY()-botPickup, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();

        yellow1drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark1.getX(),sampleMark1.getY()-botPickup, Point.CARTESIAN), new Point(dropposition.getX(),dropposition.getY(), Point.CARTESIAN)))
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
                .setPathEndTimeoutConstraint(0)
                .build();

        yellow3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropposition.getX(), dropposition.getY(),Point.CARTESIAN), new Point(dropposition.getX()+32, dropposition.getY()+24,Point.CARTESIAN), new Point(sampleMark3.getX()+16,sampleMark3.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(dropposition.getHeading(),sampleMark3.getHeading(),0.5)
                .addPath(new BezierLine(new Point(sampleMark3.getX()+16,sampleMark3.getY(), Point.CARTESIAN), new Point(sampleMark3.getX()+botPickup,sampleMark3.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(sampleMark3.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        yellow3drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark3.getX()+botPickup,sampleMark3.getY(), Point.CARTESIAN), new Point(dropposition.getX(),dropposition.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(sampleMark3.getHeading(),dropposition.getHeading(),0.5)
                .setPathEndTimeoutConstraint(0)
                .build();

        ascent = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropposition.getX(), dropposition.getY(),Point.CARTESIAN), new Point(ascentPose.getX(),ascentPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(dropposition.getHeading(),ascentPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(0)
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
                if (pathTimer.getElapsedTime()>500) {
                follower.followPath(fwdHighCmbr);

                setPathState(11);
                }

                break;

            case 11: // Yellow1
                if (follower.getPose().getY() > highchamber.getY()-1 && pathTimer.getElapsedTime() > 1000) {
                    robot.armPos(robot.armHighCa - 300, 0.4);
                    if (pathTimer.getElapsedTime() > 1500) {
                        robot.openClaw();
                    }

                    follower.followPath(yellow1);

                    setPathState(12);
                }

                break;

            case 12: //Yellow1pick
                if (pathTimer.getElapsedTime() > 100) { // atParametricEnd()
                    robot.pickSample();

                    if (armSetDown() && pathTimer.getElapsedTime() > 1000) { // && pathTimer.getElapsedTime() > 2000
                        robot.closeClaw();

                        setPathState(13);
                    }
                }

                    break;

            case 13: //Yellow1drop
                if (pathTimer.getElapsedTime() > 100) {
                    robot.rearBasket();

                    follower.followPath(yellow1drop);

                    if (armSetUp()) { //atPathEnd() && 
                        robot.openClaw();
                        setPathState(14);
                    }
                }

                break;

            case 14: //Yellow2pick
                if (pathTimer.getElapsedTime() > 300) {
                    robot.pickSample();

                    follower.followPath(yellow2);

                    if (armSetDown() && pathTimer.getElapsedTime()>2500) { //atPathEnd() && armSetDown() &&
                        robot.closeClaw();

                        setPathState(15);
                    }
                }

                break;

            case 15: //Yellow2drop
                if (pathTimer.getElapsedTime() > 100) {
                    robot.rearBasket();

                    follower.followPath(yellow2drop);

                    if (atPathEnd() && armSetUp()) { //isAtEndOfPathAndNotMoving()

                        robot.openClaw();

                        setPathState(16);
                    }
                }

                    break;

            case 16: //Yellow3pick
                if (pathTimer.getElapsedTime() > 300) {
                    robot.moveWristHighBw();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                    robot.openClaw();

                    follower.followPath(yellow3);

                    if (follower.atParametricEnd() && pathTimer.getElapsedTime()>3000) { //atPathEnd() &&
                        robot.closeClaw();

                        setPathState(17);
                    }
                }

                    break;

            case 17: //Yellow3drop
                if (pathTimer.getElapsedTime() > 100) {
                    robot.rearBasket();

                    follower.followPath(yellow3drop);

                    if (atPathEnd() && armSetUp() && pathTimer.getElapsedTime()>2000) { //isAtEndOfPathAndNotMoving()

                        robot.openClaw();

                        setPathState(18);
                    }
                }

                    break;

            case 18: //parking
                if (pathTimer.getElapsedTime() > 300) {

                    robot.closeClaw();
                    robot.moveWristCarry();
                    robot.armPos(robot.armAscent, robot.armEPower);
                    robot.extArmPos(0, robot.extArmEPower);

                    follower.followPath(ascent);

                    if (atPathEnd() && pathTimer.getElapsedTime()>2500) {

                        setPathState(100);
                    }
                }

                    break;

            case 100:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.holdPoint(ascentPose);
                }
                break;

            default:
                requestOpModeStop();
                break;

        }
        if (opmodeTimer.getElapsedTimeSeconds() > 28 && robot.arm.getCurrentPosition()<robot.armAscent-10) {
            robot.collapse();
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

    public void setPathState (int state) {
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
