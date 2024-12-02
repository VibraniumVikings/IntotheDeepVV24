package org.firstinspires.ftc.teamcode.Auton;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Core.vvHardwareITDPedro;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;


// Pedro Auton with 1 high chamber, 3 high baskets and park

/*
 * High Basket Sequence
 * Start the robot with the right side on the X tile line against the wall-
 * Path: Represents a single movement, which can be a curve (BezierCurve) or a straight line (BezierLine).
 * PathChain: Contains Path(s) within it.
 */

//TODO Create an class ?
	/*  1) changed Pose position during testing for example changed HighChamber yPos to half so that
		 robot should not hit the High chmaber area
		2) Testing of High chamber for multiple iteratons to ensure High chmaber works
		3) Testing of Yellow 1 sample for multiple iteratons to ensure code works, push the changes
		4) Testing of Yellow 2 sample for multiple iteratons to ensure code works, push the changes
		5) Testing of Yellow 3 sample for multiple iteratons to ensure code works, push the changes
		6) Remove delay added in the code for ElapsedTime

	*/

@Autonomous(name = "vvBasketPedro2", group = "1 - Auton", preselectTeleOp="vvTeleOp")


public class vvBasketPedro2 extends OpMode {

    private vvHardwareITDPedro robot;

    //private final ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, opmodeTimer;

    private Follower follower;

    private Path pathFwdHighCmbr;

    private PathChain pathchainYellow1, pathchainYellow1drop;
    private PathChain pathchainYellow2, pathchainYellow2drop;
    private PathChain pathchainYellow3, pathchainYellow3drop;
    private PathChain pathchainascent;

    private ePathState pathState;

    // We want to start the bot at x: 14, y: -60, heading: 90 degrees (these are FIRST coordinates)
    private final Pose startPose = new Pose(65, 7, Math.toRadians(90));
    // all sample mark locations
    private final Pose highchamberPose = new Pose(65,33.5);
    private final Pose sampleMark1Pose = new Pose(22.5,43);
    private final Pose sampleMark2Pose = new Pose(12.5,43);
    private final Pose sampleMark3Pose = new Pose(2.5,43,Math.toRadians(180));
    private final Pose droppositionPose = new Pose (16,16,Math.toRadians(45));
    //private final Pose specimenMark1Pose = new Pose(121.5, 43);
    //private final Pose specimenMark2Pose = new Pose(131.5, 43);
    //private final Pose specimenMark3Pose = new Pose(141.5, 43,Math.toRadians(0));
    private final Pose ascentPose  = new Pose(72, 72, Math.toRadians(0));
    //Kraken dimensional offsets
    public double botWidth = 7;
    public double botLength = 7;
    public double botPickup = 11;

    private enum ePathState{
        IntialPosition,
        MoveArmToPostion,
        HighChamberArea,
        StartHighChamberSpecimenPlacement,
        CheckHighChamberSpecimenPlacementDone,

        //Yellow 1 Sample
        MoveToYellowSample1Mark,
        CheckYellowSampleMark1Reached,
        StartYellowSample1Pick,
        CheckYellowSample1PickDone,
        MovetoDropPosforYellow1SampleDrop,
        CheckDropPosRechedforYellow1SampleDrop,
        StartYellowSample1Drop,
        CheckYellowSample1DropDone,

        //Yellow 2 Sample
        MoveToYellowSample2Mark,
        CheckYellowSampleMark2Reached,
        StartYellowSample2Pick,
        CheckYellowSample2PickDone,
        MovetoDropPosforYellow2SampleDrop,
        CheckDropPosReachedforYellow2SampleDrop,
        StartYellowSample2Drop,
        CheckYellowSample2DropDone,

        //Yellow 3 Sample
        MoveToYellowSample3Mark,
        CheckYellowSampleMark3Reached,
        StartYellowSample3Pick,
        CheckYellowSample3PickDone,
        MovetoDropPosforYellow3SampleDrop,
        CheckDropPosRechedforYellow3SampleDrop,
        StartYellowSample3Drop,
        CheckYellowSample3DropDone,

        //Parking
        InitParking,
        MovetoParking,
        CheckParkingZoneReached,
        StopinParkingZone
    }

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

        //High Chamber travel path Line path - startpostion-> High chamber
        pathFwdHighCmbr = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                new Point(highchamberPose.getX(), highchamberPose.getY(), Point.CARTESIAN)
        )); //Tile Start Position
        pathFwdHighCmbr.setConstantHeadingInterpolation(startPose.getHeading());
        pathFwdHighCmbr.setPathEndTimeoutConstraint(2); //2 ms?

        //Path from submersible to the first yellow sample, approaches straight on after a sweep
        pathchainYellow1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(highchamberPose.getX(), highchamberPose.getY(),Point.CARTESIAN),
                        new Point(47,16, Point.CARTESIAN),
                        new Point(sampleMark1Pose.getX(),sampleMark1Pose.getY()-botPickup, Point.CARTESIAN
                        )))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();

        pathchainYellow1drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark1Pose.getX(),sampleMark1Pose.getY()-botPickup, Point.CARTESIAN),
                        new Point(droppositionPose.getX(),droppositionPose.getY(), Point.CARTESIAN
                        )))
                .setLinearHeadingInterpolation(startPose.getHeading(),droppositionPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        pathchainYellow2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(droppositionPose.getX(), droppositionPose.getY(),Point.CARTESIAN),
                        new Point(sampleMark2Pose.getX(),sampleMark2Pose.getY()-botPickup, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(droppositionPose.getHeading(),startPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        pathchainYellow2drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark2Pose.getX(),sampleMark2Pose.getY()-botPickup, Point.CARTESIAN),
                        new Point(droppositionPose.getX(),droppositionPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),droppositionPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(0)
                .build();

        pathchainYellow3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(droppositionPose.getX(), droppositionPose.getY(),Point.CARTESIAN),
                        new Point(droppositionPose.getX()+32, droppositionPose.getY()+24,Point.CARTESIAN),
                        new Point(sampleMark3Pose.getX()+16,sampleMark3Pose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(droppositionPose.getHeading(),sampleMark3Pose.getHeading(),0.5)
                .addPath(new BezierLine(new Point(sampleMark3Pose.getX()+16,sampleMark3Pose.getY(), Point.CARTESIAN),
                        new Point(sampleMark3Pose.getX()+botPickup,sampleMark3Pose.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(sampleMark3Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        pathchainYellow3drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleMark3Pose.getX()+botPickup,sampleMark3Pose.getY(), Point.CARTESIAN),
                        new Point(droppositionPose.getX(),droppositionPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(sampleMark3Pose.getHeading(),droppositionPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(0)
                .build();

        pathchainascent = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(droppositionPose.getX(), droppositionPose.getY(),Point.CARTESIAN),
                        new Point(ascentPose.getX(),ascentPose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(droppositionPose.getHeading(),ascentPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState (ePathState state) {
        pathState = state;
        pathTimer.resetTimer();
        //autonPathUpdate(); //TODO Purpose
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method resetTimer)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonPathUpdate() {
        switch (pathState) {

            case IntialPosition:
                follower.followPath(pathFwdHighCmbr); //follow the path for forward High chamber
                telemetry.addData("Status", "Follow path for High chamber");
                setPathState(ePathState.MoveArmToPostion);
                break;

            case MoveArmToPostion:
                //how to check follower start
                //if(!follower.isBusy()  //recommendation not to use
                if (pathTimer.getElapsedTime() > 1) { // checking the path timer running timer restarts
                    robot.rgb.setPosition(0.5); //TODO Purpose ?
                    telemetry.addData("Status", "MoveArm armPos , moveWritstHighClw");
                    robot.armPos(robot.armHighCa, robot.armEPower);
                    robot.moveWristHighCw();
                    setPathState(ePathState.HighChamberArea);
                }
                break;

            case HighChamberArea: //make sure that robot reach High chamber area
                //TODO armPos and movemoveWristHighCw is blocking call or not then add elapsedtime
                if (follower.getPose().getY() > highchamberPose.getY() - 1
                        && follower.getPose().getX() > highchamberPose.getX() - 1 ) {
                    telemetry.addData("Status", "High Chamber area");
                    robot.armPos(robot.armHighCa - 300, 0.4);
                    setPathState(ePathState.StartHighChamberSpecimenPlacement);
                }
                else
                    telemetry.addData("Status", "Not in High Chamber area");
                break;

            case StartHighChamberSpecimenPlacement:
                //TODO Extra check we can remove this after testing
                //TODO check armPos is blocking call or not then add elapsedtime
                if (follower.getPose().getY() > highchamberPose.getY()-1
                        && follower.getPose().getX() > highchamberPose.getX()-1 ) {
                    telemetry.addData("Status", "High Chamber placement openClaw");
                    robot.openClaw(); //High specimen placement
                    setPathState(ePathState.CheckHighChamberSpecimenPlacementDone);
                }
                else
                    telemetry.addData("Status", "Not in High Chamber area openClaw not set");
                break;

            case CheckHighChamberSpecimenPlacementDone:
                //TODO reduce the delay during testing
                if (pathTimer.getElapsedTime() > 100) {
                    telemetry.addData("Status", "HighChamberSpecimentPlacement done after 100 ms");
                    setPathState(ePathState.MoveToYellowSample1Mark);
                }
                else
                    telemetry.addData("Status", "Delay , checking HighChamberSpecimentPlacement done");
                break;

            case MoveToYellowSample1Mark:
                if (pathTimer.getElapsedTime() > 1) {
                    telemetry.addData("Status", "Follow path for Yellow1 Sample");
                    follower.followPath(pathchainYellow1);
                    setPathState(ePathState.CheckYellowSampleMark1Reached);
                }
                break;

            case CheckYellowSampleMark1Reached: //check robot reach Yellow sample 1
                if (follower.getPose().getY() > sampleMark1Pose.getY()-1
                        && follower.getPose().getX() > sampleMark1Pose.getX()-1 ){
                    telemetry.addData("Status", "YellowSampleMark1Reached");
                    setPathState(ePathState.StartYellowSample1Pick);
                }
                else
                    telemetry.addData("Status", "Not reached Yellow1 Sample pos");
                break;

            case StartYellowSample1Pick:
                if (follower.getPose().getY() > sampleMark1Pose.getY()-1
                        && follower.getPose().getX() > sampleMark1Pose.getX()-1 ){ //check is require?
                    robot.pickSample(); //not sure this blocking call
                    telemetry.addData("Status", "Pick Yellow Sample 1");
                    robot.closeClaw();
                    telemetry.addData("Status", "Close claw for Yellow Sample 1 Pick");
                    setPathState(ePathState.CheckYellowSample1PickDone);
                }
                else
                    telemetry.addData("Status", "Not reached StartYellowSample1Pick");
                break;

            case CheckYellowSample1PickDone:
                //TODO wait for some time to perform task in StartYellowSamplePick
                //if the task is blocking call remove the delay
                if (pathTimer.getElapsedTime() > 50) {
                    telemetry.addData("Status", "Pick Yellow Sample 1 : rearbasket");
                    robot.rearBasket();
                    setPathState(ePathState.MovetoDropPosforYellow1SampleDrop);
                }
                else
                    telemetry.addData("Status", "Elapsed time for CheckYellowSample1PickDone 50 ms");
                break;

            case MovetoDropPosforYellow1SampleDrop:
                //TODO wait for some time to perform task for rearBasket
                if (pathTimer.getElapsedTime() > 50) {
                    telemetry.addData("Status", "follower path for pathchainYellow1drop");
                    follower.followPath(pathchainYellow1drop);
                    setPathState(ePathState.CheckDropPosRechedforYellow1SampleDrop);
                }
                else
                    telemetry.addData("Status", "Elapsed time for MovetoDropPosforYellow1SampleDrop 50 ms");
                break;

            case CheckDropPosRechedforYellow1SampleDrop:
                if (follower.getPose().getY() > droppositionPose.getY()-1
                        && follower.getPose().getX() > droppositionPose.getX()-1 ){
                    telemetry.addData("Status", "Set StartYellowSample1Drop");
                    setPathState(ePathState.StartYellowSample1Drop);
                }
                else
                    telemetry.addData("Status", "Not reached to Drop Position for Yellow1 Sample ");
                break;

            case StartYellowSample1Drop:
                //TODO sequence is correct
                robot.rearBasket();
                if (atPathEnd() && armSetUp()) { //isAtEndOfPathAndNotMoving()
                    telemetry.addData("Status", "Open claw for StartYellowSample1Drop");
                    robot.openClaw();
                    telemetry.addData("Status", "Set CheckYellowSample1DropDone");
                    setPathState(ePathState.CheckYellowSample1DropDone);
                }
                else
                    telemetry.addData("Status", "CheckYellowSample1DropDone path not set ");
                break;

            case CheckYellowSample1DropDone:
                //TODO delay for tasks in StartYellowSample1Drop
                if (pathTimer.getElapsedTime() > 100) {
                    telemetry.addData("Status", "Set CheckYellowSample1DropDone");
                    setPathState(ePathState.MoveToYellowSample2Mark);
                }
                else
                    telemetry.addData("Status", "Elpased time for CheckYellowSample1DropDone 100 ");
                break;

            case MoveToYellowSample2Mark:
                if (pathTimer.getElapsedTime() > 1) {
                    telemetry.addData("Status", "follow path for pathchainYellow2e");
                    follower.followPath(pathchainYellow2);
                    setPathState(ePathState.CheckYellowSampleMark2Reached);
                }
                else
                    telemetry.addData("Status", "Elpased time for MoveToYellowSample2Mark 1ms ");
                break;

            case CheckYellowSampleMark2Reached: //check robot reach Yellow sample 1
                if (follower.getPose().getY() > sampleMark2Pose.getY()-1
                        && follower.getPose().getX() > sampleMark2Pose.getX()-1 ){
                    telemetry.addData("Status", "Set CheckYellowSampleMark2Reached");
                    setPathState(ePathState.StartYellowSample2Pick);
                }
                else
                    telemetry.addData("Status", "Not reached Yellow2 Sample pos");
                break;

            case StartYellowSample2Pick:
                if (follower.getPose().getY() > sampleMark2Pose.getY()-1
                        && follower.getPose().getX() > sampleMark2Pose.getX()-1 ){
                    telemetry.addData("Status", "pickSample for StartYellowSample2Pick");
                    robot.pickSample(); //not sure this blocking call
                    telemetry.addData("Status", "closeClaw for StartYellowSample2Pick");
                    robot.closeClaw();
                    telemetry.addData("Status", "Set CheckYellowSample2PickDone");
                    setPathState(ePathState.CheckYellowSample2PickDone);
                }
                else
                    telemetry.addData("Status", "CheckYellowSample2PickDone not set");
                break;

            case CheckYellowSample2PickDone:
                //TODO delay for tasks in StartYellowSample2Pick
                if (pathTimer.getElapsedTime() > 50) { //wait for some time //reduce the delay
                    telemetry.addData("Status", "rearBasket for CheckYellowSample2PickDone");
                    robot.rearBasket();
                    setPathState(ePathState.MovetoDropPosforYellow2SampleDrop);
                }
                else
                    telemetry.addData("Status", "Elapsed time for CheckYellowSample2PickDone 50");
                break;

            case MovetoDropPosforYellow2SampleDrop:
                //TODO elapsed time for rearBasket in CheckYellowSample2PickDone
                if (pathTimer.getElapsedTime() > 50) {
                    telemetry.addData("Status", "follower path for pathchainYellow2drop");
                    follower.followPath(pathchainYellow2drop);
                    setPathState(ePathState.CheckDropPosReachedforYellow2SampleDrop);
                }
                else
                    telemetry.addData("Status", "Elapsed time for MovetoDropPosforYellow2SampleDrop 50ms");
                break;

            case CheckDropPosReachedforYellow2SampleDrop:
                if (follower.getPose().getY() > droppositionPose.getY()-1
                        && follower.getPose().getX() > droppositionPose.getX()-1 ){
                    telemetry.addData("Status", "set StartYellowSample2Drop");
                    setPathState(ePathState.StartYellowSample2Drop);
                }
                else
                    telemetry.addData("Status", "Not reached to Drop Postion for Yellow2 Sample ");
                break;
            case StartYellowSample2Drop:
                if (pathTimer.getElapsedTime() > 1) {
                    telemetry.addData("Status", "rearBasket for StartYellowSample2Drop");
                    robot.rearBasket();
                    if (atPathEnd() && armSetUp()) { //isAtEndOfPathAndNotMoving()
                        telemetry.addData("Status", "openClaw for StartYellowSample2Drop");
                        robot.openClaw();
                        telemetry.addData("Status", "set CheckYellowSample2DropDone");
                        setPathState(ePathState.CheckYellowSample2DropDone);
                    }
                    else
                        telemetry.addData("Status", "StartYellowSample2Drop pathend and armsetup error ");
                }
                break;
            case CheckYellowSample2DropDone:
                //TODO elasped time for tasks in StartYellowSample2Drop check blocking call
                if (pathTimer.getElapsedTime() > 50) {
                    telemetry.addData("Status", "set MoveToYellowSample3Mark");
                    setPathState(ePathState.MoveToYellowSample3Mark);
                }
                else
                    telemetry.addData("Status", "Elasped time for CheckYellowSample2DropDone 50 ms ");
                break;

            case MoveToYellowSample3Mark:
                if (pathTimer.getElapsedTime() > 1) {
                    telemetry.addData("Status", "follower path for pathchainYellow3");
                    follower.followPath(pathchainYellow3);
                    telemetry.addData("Status", "set for CheckYellowSampleMark3Reached");
                    setPathState(ePathState.CheckYellowSampleMark3Reached);
                }
                else
                    telemetry.addData("Status", "Elasped time for MoveToYellowSample3Mark 1 ms ");
                break;

            case CheckYellowSampleMark3Reached: //check robot reach Yellow sample 1
                if (follower.getPose().getY() > sampleMark3Pose.getY()-1
                        && follower.getPose().getX() > sampleMark3Pose.getX()-1 ){
                    telemetry.addData("Status", "set CheckYellowSampleMark3Reached");
                    setPathState(ePathState.StartYellowSample3Pick);
                }
                else
                    telemetry.addData("Status", "Not reached Yellow3 Sample pos");
                break;

            case StartYellowSample3Pick:
                if (follower.getPose().getY() > sampleMark3Pose.getY()-1
                        && follower.getPose().getX() > sampleMark3Pose.getX()-1 ){
                    telemetry.addData("Status", "pickSample() for StartYellowSample3Pick");
                    robot.pickSample(); //not sure this blocking call
                    telemetry.addData("Status", "closeClaw for StartYellowSample3Pick");
                    robot.closeClaw();
                    telemetry.addData("Status", "set CheckYellowSample3PickDone");
                    setPathState(ePathState.CheckYellowSample3PickDone);
                }
                else
                    telemetry.addData("Status", "Not reached StartYellowSample3Pick");
                break;

            case CheckYellowSample3PickDone:
                //TODO check delay is correct for the tasks in StartYellowSample3Pick
                if (pathTimer.getElapsedTime() > 55) { //wait for some time //reduce the delay
                    telemetry.addData("Status", "rearBasket for CheckYellowSample3PickDone");
                    robot.rearBasket();
                    telemetry.addData("Status", "set for MovetoDropPosforYellow3SampleDrop");
                    setPathState(ePathState.MovetoDropPosforYellow3SampleDrop);
                }
                else
                    telemetry.addData("Status", "Elapsed time for CheckYellowSample3PickDone 55 ms");
                break;

            case MovetoDropPosforYellow3SampleDrop:
                if (pathTimer.getElapsedTime() > 50) {
                    telemetry.addData("Status", "follower path for pathchainYellow3drop");
                    follower.followPath(pathchainYellow3drop);
                    telemetry.addData("Status", "set CheckDropPosRechedforYellow3SampleDrop");
                    setPathState(ePathState.CheckDropPosRechedforYellow3SampleDrop);
                }
                else
                    telemetry.addData("Status", "Elapsed time for MovetoDropPosforYellow3SampleDrop 50 ms");
                break;

            case CheckDropPosRechedforYellow3SampleDrop:
                if (follower.getPose().getY() > droppositionPose.getY()-1
                        && follower.getPose().getX() > droppositionPose.getX()-1 ){
                    telemetry.addData("Status", "set StartYellowSample3Drop");
                    setPathState(ePathState.StartYellowSample3Drop);
                }
                else
                    telemetry.addData("Status", "Not reached to Drop Postion for Yellow3 Sample ");
                break;

            case StartYellowSample3Drop:
                if (pathTimer.getElapsedTime() > 2) {
                    telemetry.addData("Status", "rearBasket for StartYellowSample3Drop");
                    robot.rearBasket();
                    if (atPathEnd() && armSetUp()) { //isAtEndOfPathAndNotMoving()
                        telemetry.addData("Status", "openClaw for StartYellowSample3Drop");
                        robot.openClaw();
                        telemetry.addData("Status", "set CheckYellowSample3DropDone");
                        setPathState(ePathState.CheckYellowSample3DropDone);
                    }
                    else
                        telemetry.addData("Status", "pathend and armSetup error for StartYellowSample3Drop");
                }
                else
                    telemetry.addData("Status", "Elapsed time for StartYellowSample3Drop 2 ms");
                break;

            case CheckYellowSample3DropDone:
                //TODO checked delay for tasks in StartYellowSample3Drop
                if (pathTimer.getElapsedTime() > 45) {
                    setPathState(ePathState.InitParking);
                }
                else
                    telemetry.addData("Status", "Elapsed time for CheckYellowSample3DropDone 45 ms");
                break;

            case InitParking:
                if (pathTimer.getElapsedTime() > 1) {
                    telemetry.addData("Status", "closeClaw for InitParking");
                    robot.closeClaw();
                    telemetry.addData("Status", "moveWristCarry for InitParking");
                    robot.moveWristCarry();
                    telemetry.addData("Status", "armPos for InitParking");
                    robot.armPos(robot.armAscent, robot.armEPower);
                    telemetry.addData("Status", "extArmPos for InitParking");
                    robot.extArmPos(0, robot.extArmEPower);
                    telemetry.addData("Status", "Set MovetoParking");
                    setPathState(ePathState.MovetoParking);
                }
                break;
            case MovetoParking:
                if (pathTimer.getElapsedTime() > 100) { //wait for some time, to finish Initparking tasks need to check delay
                    telemetry.addData("Status", "closeClaw for MovetoParking");
                    robot.closeClaw();
                    telemetry.addData("Status", "follower path for pathchainascent");
                    follower.followPath(pathchainascent);
                    telemetry.addData("Status", "set CheckParkingZoneReched");
                    setPathState(ePathState.CheckParkingZoneReached);
                }
                else
                    telemetry.addData("Status", "Elapsed time for MovetoParking 100 ms");
                break;

            case CheckParkingZoneReached :
                if (follower.getPose().getY() > ascentPose.getY()-1
                        && follower.getPose().getX() > ascentPose.getX()-1 ){
                    telemetry.addData("Status", "set StopinParkingZone");
                    setPathState(ePathState.StopinParkingZone);
                }
                else
                    telemetry.addData("Status", "Not reached parking zone");
                break;

            case StopinParkingZone:
                if (pathTimer.getElapsedTime() > 2) {
                    telemetry.addData("Status", "Follower hold point for StopinParkingZone");
                    follower.holdPoint(ascentPose);
                }
                break;

            default:
                break;
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        follower.update();
        autonPathUpdate();
        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init() {

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



    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        resetRuntime();
        setPathState(ePathState.IntialPosition);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    }
