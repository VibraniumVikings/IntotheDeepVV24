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
@Autonomous(name = "vvChamberPedro", group = "2 - Auton", preselectTeleOp="vvNewArm")

public class vvChamberPedro extends OpMode {
    private vvHardwareITDPedro robot;

    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, opmodeTimer;

    private Follower follower;

    private Path fwdHighCmbr;

    private PathChain sample1, sample2, sample3, turnAround, pickTA, samplePick, samplePlace1, samplePlace2, samplePlace3, obsPick2, obsPick3, obsEnd;

    private int pathState;

    // We want to start the bot at x: 79, y: 7, heading: 90 degrees
    private Pose startPose = new Pose(79, 7, Math.toRadians(90));
    // all sample mark locations
    private Pose highchamber = new Pose(78,36);
    private Pose sampleMark1 = new Pose(26,44);
    private Pose sampleMark2 = new Pose(15,44);
    private Pose sampleMark3 = new Pose(4.5,57,Math.toRadians(180));
    private Pose dropposition = new Pose (18,21,Math.toRadians(45));
    private Pose dropposition2 = new Pose (15,21, Math.toRadians(60));
    private Pose ascentPose  = new Pose(78,85, Math.toRadians(5));
    //Chamber movement poses
    private Pose sub1 = new Pose(106, 48, Math.toRadians(90));
    private Pose sub1Cntrl1 = new Pose(128, 4);
    private Pose sub2 = new Pose(113, 60, Math.toRadians(90));
    private Pose sub1Cntrl2 = new Pose(115, 138);

    private Pose spec1Cntrl = new Pose(126, 72);
    private Pose obsZone1 = new Pose(120, 18, Math.toRadians(90));
    private Pose spec2 = new Pose(126, 60);
    private Pose spec2Cntrl1 = new Pose(96, 48);
    private Pose spec2Cntrl2 = new Pose(136, 105);
    private Pose obsZone2 = new Pose(130, 18, Math.toRadians(90));
    private Pose turn = new Pose(115, 32, Math.toRadians(-91));

    private Pose spec3 = new Pose(115, 45, Math.toRadians(1));
    private Pose spec3Cntrl = new Pose(91, 47);
    private Pose obsZone3 = new Pose(100, 25, Math.toRadians(-91));
    private Pose obsCntrl = new Pose(60, 16);
    private Pose obsInt = new Pose(72, 28,Math.toRadians(45));
    private Pose chmbrCntrl = new Pose(72, 0);
    private Pose chmbrInt = new Pose(72, 28);
    //Specimen and previous positions for reference
    private Pose specimenMark1 = new Pose(120, 43, Math.toRadians(90));
    private Pose specimenMark2 = new Pose(130, 43, Math.toRadians(90));
    private Pose specimenMark3 = new Pose(140, 43,Math.toRadians(0));
    private Pose specimenMark1Control = new Pose (110,60);
    private Pose specimenMark11Control = new Pose (120,72); //116,60
    private Pose specimenMark2Control = new Pose (114,60);
    private Pose specimenMark21Control = new Pose (148,72); //118,60
    private Pose obsControl= new Pose(106,48, Math.toRadians(-90)); //spin around point
    private Pose observationZone= new Pose(110,10, Math.toRadians(-89));

    //Kraken dimensional offsets
    public double botWidth = 7;
    public double botLength = 11; //prev 7
    public double botPickup = 18;

    public boolean isHalfwayThere() {
        return follower.getCurrentTValue() > 0.75;
    }
    public boolean atPathEnd() {
        return follower.getCurrentTValue() > 0.99;
    }
    public boolean isAtEndOfPathAndNotMoving() {
        return atPathEnd() && follower.getVelocityMagnitude() < 0.1;
    }
    public boolean armSetUp() {
        return robot.arm.getCurrentPosition()>robot.arm.getTargetPosition()-10;
    }
    public boolean armSetDown() {
        return robot.arm.getCurrentPosition()>robot.arm.getTargetPosition()+5;
    }

    public void buildPaths() {

        fwdHighCmbr = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), 
                                              new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN))); //Tile Start Position
        fwdHighCmbr.setConstantHeadingInterpolation(startPose.getHeading());
        fwdHighCmbr.setPathEndTimeoutConstraint(2);

        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(highchamber),
                        new Point(sub1Cntrl1),
                        new Point(sub1)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierCurve(new Point(sub1),
                        //new Point(sub1Cntrl2),
                        new Point(sub2)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                //.addPath(new BezierCurve(new Point(sub2),
                        //new Point(spec1Cntrl),
                        //new Point(obsZone1)))
                .addPath(new BezierLine(new Point(sub2),
                        new Point(obsZone1)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(7)
                .build();

        sample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obsZone1),
                        new Point(spec2Cntrl1),
                        //new Point(spec2Cntrl2),
                        new Point(spec2)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(spec2),
                        new Point(obsZone2)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        turnAround = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obsZone2),
                        new Point(turn)))
                .setLinearHeadingInterpolation(startPose.getHeading(),turn.getHeading(),0.5)
                .addPath(new BezierLine(new Point(turn),
                        new Point(obsZone3)))
                .setConstantHeadingInterpolation(obsZone3.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();

        sample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obsZone2),
                        new Point(spec3Cntrl),
                        new Point(spec3)))
                .setLinearHeadingInterpolation(startPose.getHeading(),spec3.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        samplePick = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spec3),
                                        new Point(obsZone3)))
                .setLinearHeadingInterpolation(spec3.getHeading(),obsZone3.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        samplePlace1 = follower.pathBuilder()
                //.addPath(new BezierCurve(new Point(obsZone3),
                                        //new Point(chmbrCntrl),
                                        //new Point(highchamber.getX()-4, highchamber.getY(), Point.CARTESIAN)))
                //.setLinearHeadingInterpolation(obsZone3.getHeading(),startPose.getHeading(),0.5)
                .addPath(new BezierLine(new Point(obsZone3),
                        new Point(chmbrInt)))
                .setLinearHeadingInterpolation(obsZone3.getHeading(),startPose.getHeading())
                .addPath(new BezierLine(new Point(chmbrInt),
                        new Point(highchamber.getX()-4, highchamber.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        obsPick2 = follower.pathBuilder()
                //.addPath(new BezierCurve(new Point(highchamber.getX()-4, highchamber.getY(), Point.CARTESIAN),
                  //                      new Point(obsCntrl),
                    //                    new Point(obsZone3)))
                //.setLinearHeadingInterpolation(startPose.getHeading(),obsZone3.getHeading(),0.5)
                .addPath(new BezierLine(new Point(highchamber.getX()-4, highchamber.getY(), Point.CARTESIAN),
                        new Point(obsInt)))
                .setLinearHeadingInterpolation(startPose.getHeading(),obsInt.getHeading())
                .addPath(new BezierLine(new Point(obsInt),
                        new Point(obsZone3)))
                .setLinearHeadingInterpolation(obsInt.getHeading(),obsZone3.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        samplePlace2 = follower.pathBuilder()
                //.addPath(new BezierCurve(new Point(obsZone3),
                  //      new Point(chmbrCntrl),
                  //      new Point(highchamber.getX()-8, highchamber.getY(), Point.CARTESIAN)))
                //.setLinearHeadingInterpolation(obsZone3.getHeading(),startPose.getHeading(),0.5)
                .addPath(new BezierLine(new Point(obsZone3),
                        new Point(chmbrInt)))
                .setLinearHeadingInterpolation(obsZone3.getHeading(),startPose.getHeading())
                .addPath(new BezierLine(new Point(chmbrInt),
                        new Point(highchamber.getX()-8, highchamber.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        obsPick3 = follower.pathBuilder()
                //.addPath(new BezierCurve(new Point(highchamber.getX()-8, highchamber.getY(), Point.CARTESIAN),
                  //      new Point(obsCntrl),
                  //      new Point(obsZone3)))
                //.setLinearHeadingInterpolation(startPose.getHeading(),obsZone3.getHeading(),0.5)
                .addPath(new BezierLine(new Point(highchamber.getX()-8, highchamber.getY(), Point.CARTESIAN),
                        new Point(obsZone3.getX(),obsZone3.getY()-8,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),Math.toRadians(0))
                //.addPath(new BezierLine(new Point(obsInt),
                  //      new Point(obsZone3)))
                //.setLinearHeadingInterpolation(obsInt.getHeading(),Math.toRadians(0),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        samplePlace3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obsZone3),
                        new Point(chmbrCntrl),
                        new Point(highchamber.getX()-12, highchamber.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(obsZone3.getHeading(),startPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(5)
                .build();

        obsEnd = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(highchamber.getX()-12, highchamber.getY(), Point.CARTESIAN),
                        new Point(obsCntrl),
                        new Point(obsZone3)))
                .setLinearHeadingInterpolation(startPose.getHeading(),obsZone3.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();
    }

    public void autonPathUpdate() {
        switch (pathState) {
            case 6: //move arm to position and high chamber specimen placment
                if (pathTimer.getElapsedTime() > 10) {
                    robot.rgb.setPosition(0.5);
                    robot.armPos(robot.armHighCaNew, robot.armEPower);
                    robot.moveWristHighCwNew();
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                    setPathState(7);
                }
                break;

            case 7: //high chamber path
                if (pathTimer.getElapsedTime() > 750) {
                    follower.followPath(fwdHighCmbr,/* holdEnd = */ true);

                    setPathState(8);
                }
                break;

            case 8: // High chamber open claw
                if (pathTimer.getElapsedTime() > 1000) { //follower.getPose().getX() > (highchamber.getX() - 1) && follower.getPose().getY() > (highchamber.getY() - 1)
                    robot.claw.setPosition(0.7);

                    //robot.moveWristFloor();
                    setPathState(9);
                }

                break;
            case 9: // High chamber retract arm
                if (pathTimer.getElapsedTime() > 250) {
                    robot.extArmPos(0, robot.extArmEPower);
                    robot.armPos(robot.armHighCaNew-150, robot.armEPower);

                    setPathState(10);
                }
                break;

            case 10: // yellow1 path
                if (pathTimer.getElapsedTime() > 750) {
                    follower.followPath(sample1,/* holdEnd = */ true);

                    setPathState(11);
                }

                break;


            case 11: // Sample 1 push to obszone
                if (pathTimer.getElapsedTime() > 1500) {
                    robot.collapse();
                    telemetry.addData("Status", "Sample1 push to observe");
                    setPathState(12);
                }

                break;

            case 12: // Sample 2 push
                if (follower.getPose().getY() < (obsZone1.getY() + 2) || pathTimer.getElapsedTime() > 5000) { //follower.getPose().getY() < (obsZone1.getY() + 2) ||
                    follower.followPath(sample2,/* holdEnd = */ true);

                    setPathState(125);
                }

                break;

            case 125: // Specimen 3 positioning and arm to position
                if (follower.getPose().getY() < (obsZone2.getY() + 2) && pathTimer.getElapsedTime() > 1000) { //follower.getPose().getX() > (obsZone2.getX() - 2) ||  || pathTimer.getElapsedTime() > 3000 follower.atParametricEnd()
                    robot.wallPick();
                    follower.followPath(turnAround,/* holdEnd = */ true);
                    //follower.followPath(samplePick,/* holdEnd = */ true);
                    robot.openClaw();

                    setPathState(134);
                }
                break;

            case 13: // Specimen 3 positioning and arm to position
                if (follower.getPose().getY() < (obsZone2.getY() + 2) && pathTimer.getElapsedTime() > 1000) { //follower.getPose().getX() > (obsZone2.getX() - 2) ||  || pathTimer.getElapsedTime() > 3000 follower.atParametricEnd()
                    robot.moveWristHighBw();
                    robot.armPos(robot.floorArm, robot.armEPower);
                    robot.extArmPos(robot.extArmFLoorPick, robot.armEPower);
                    robot.openClaw();
                    follower.followPath(sample3,/* holdEnd = */ true);

                    setPathState(131);
                }
                break;

            case 131: // Specimen 3 grab and claw close
                if (follower.getPose().getX() > (spec3.getX() - 1) && pathTimer.getElapsedTime() > 1500) { //follower.getPose().getY() > (observationZone.getY() + botPickup + 0.5)
                    robot.closeClaw();

                    setPathState(132);
                }
                break;

            case 132: // To observation zone and move arm to position
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.wallPick();
                    follower.followPath(samplePick,/* holdEnd = */ true);

                    setPathState(133);
                }
                break;

            case 133: // To observation zone and drop spec3
                if (follower.getPose().getY() < (obsZone3.getY() + 1) && pathTimer.getElapsedTime() > 1000) {
                    robot.openClaw();

                    setPathState(134);
                }
                break;

            case 134: // To observation zone and grab spec
                if (follower.getPose().getY() < (obsZone3.getY()+0.25) && pathTimer.getElapsedTime() > 1500) {
                    robot.closeClaw();

                    setPathState(135);
                }
                break;

            case 135: // To observation zone and grab spec
                if (pathTimer.getElapsedTime() > 500) {
                    robot.extArmPos(0,0.4);
                    robot.armPos(robot.armHighCaNew, robot.armEPower);

                    setPathState(14);
                }
                break;

            case 14: // To chamber for spec1 placement
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(samplePlace1,/* holdEnd = */ true);

                    setPathState(142);
                }
                break;

            case 142: // Arm to Chamber placement
                if (pathTimer.getElapsedTime() > 500) {
                    robot.armPos(robot.armHighCaNew, robot.armEPower);
                    robot.moveWristHighCwNew();
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);

                    setPathState(143);
                }
                break;

            case 143: // High chamber open claw
                if (follower.getPose().getY() > (highchamber.getY() - 1) || pathTimer.getElapsedTime() > 3000) {
                    robot.openClaw();

                    setPathState(144);
                }
                break;

            case 144: // High chamber retract arm
                if (pathTimer.getElapsedTime() > 500) {
                    robot.extArmPos(0, robot.extArmEPower);
                    robot.armPos(robot.armHighCaNew-150, robot.armEPower);

                    setPathState(15);
                }
                break;

            case 15: // Back to observation zone
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(obsPick2,/* holdEnd = */ true);

                    robot.wallPick();

                    setPathState(16);
                }
                break;

            case 16: // To observation zone and drop spec3
                if (follower.getPose().getX() > (obsZone3.getX()-0.25) && follower.getPose().getY() < (obsZone3.getY()+0.25) && pathTimer.getElapsedTime() > 2000) {
                    robot.closeClaw();

                    setPathState(165);
                }
                break;

            case 165: // To observation zone and grab spec
                if (pathTimer.getElapsedTime() > 500) {
                    robot.extArmPos(0,0.4);
                    robot.armPos(robot.armHighCaNew, robot.armEPower);

                    setPathState(17);
                }
                break;

            case 17: // To chamber for spec1 placement
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.extArmPos(0,0.4);
                    follower.followPath(samplePlace2,/* holdEnd = */ true);

                    setPathState(18);
                }
                break;

            case 18: // Arm to Chamber placement
                if (pathTimer.getElapsedTime() > 500) {
                    robot.armPos(robot.armHighCaNew, robot.armEPower);
                    robot.moveWristHighCwNew();
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);

                    setPathState(19);
                }
                break;

            case 19: // High chamber open claw
                if (follower.getPose().getY() > (highchamber.getY() - 1) || pathTimer.getElapsedTime() > 3000) {
                    robot.openClaw();

                    setPathState(20);
                }
                break;

            case 20: // High chamber retract arm
                if (pathTimer.getElapsedTime() > 250) {
                    robot.extArmPos(0, robot.extArmEPower);
                    robot.armPos(robot.armHighCaNew-150, robot.armEPower);

                    setPathState(21);
                }
                break;

            case 21: // Back to observation zone
                if (pathTimer.getElapsedTime() > 750) {
                    follower.followPath(obsPick3,/* holdEnd = */ true);

                    robot.collapse();

                    setPathState(22);
                }
                break;

            case 22: // Back to observation zone
                if (pathTimer.getElapsedTime() > 3000) {


                }
                break;

            default:
                break;
    }
            /*if (opmodeTimer.getElapsedTimeSeconds() > 28) {
                robot.collapse();
            }*/
    }


    @Override
    public void loop() {

        follower.update();

        autonPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
        follower.setMaxPower(0.7);

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Robot Ready");
        telemetry.update();

    }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();

    }
    @Override
    public void start() {

                buildPaths();

                resetRuntime();

                setPathState(6);

            }
    @Override
    public void stop() {
    }
}
