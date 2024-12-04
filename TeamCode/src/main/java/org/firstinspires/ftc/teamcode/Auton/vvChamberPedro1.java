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
@Autonomous(name = "vvChamberPedro1", group = "2 - Auton", preselectTeleOp="vvTeleOp")

public class vvChamberPedro1 extends OpMode {
    private vvHardwareITDPedro robot;

    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, opmodeTimer;

    private Follower follower;

    private Path fwdHighCmbr;

    private PathChain sample1, sample2, samplePick, sample1Place, sample2Place, obs1Zone, obs2Zone;

    private int pathState;

    // We want to start the bot at x: 14, y: -60, heading: 90 degrees
    private Pose startPose = new Pose(79, 7, Math.toRadians(90));
    // all sample mark locations
    private Pose highchamber = new Pose(78,36);
    private Pose sampleMark1 = new Pose(22.5,44);
    private Pose sampleMark2 = new Pose(12.5,44);
    private Pose sampleMark3 = new Pose(2.5,43,Math.toRadians(180));
    private Pose dropposition = new Pose (16,16,Math.toRadians(45));

    private Pose specimenMark1 = new Pose(120, 43);
    private Pose specimenMark2 = new Pose(130, 43);
    private Pose specimenMark3 = new Pose(140, 43,Math.toRadians(0));
    private Pose specimenMark1Control = new Pose (110,60);
    private Pose specimenMark11Control = new Pose (120,72); //116,60
    private Pose specimenMark2Control = new Pose (114,60);
    private Pose specimenMark21Control = new Pose (148,72); //118,60
    private Pose obsControl= new Pose(106,48, Math.toRadians(-90));
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
                .addPath(new BezierCurve(new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN),
                        new Point(108, 10, Point.CARTESIAN),
                        new Point(specimenMark1Control.getX(), specimenMark1Control.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierCurve(new Point(specimenMark1Control.getX(), specimenMark1Control.getY(), Point.CARTESIAN),
                        new Point(specimenMark11Control.getX(), specimenMark11Control.getY(), Point.CARTESIAN),
                        new Point(observationZone.getX()+8,observationZone.getY()+botLength-2, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                //.addPath(new BezierLine(new Point(specimenMark11Control.getX(), specimenMark11Control.getY(), Point.CARTESIAN),
                //      new Point(observationZone.getX()-12,observationZone.getY()+botLength, Point.CARTESIAN)))
                //.setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        sample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX()+8,observationZone.getY()+botLength-2, Point.CARTESIAN),
                        new Point(specimenMark2Control.getX(),specimenMark2Control.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierCurve(new Point(specimenMark2Control.getX(),specimenMark2Control.getY(), Point.CARTESIAN),
                        new Point(specimenMark21Control.getX(),specimenMark21Control.getY(), Point.CARTESIAN),
                        new Point(observationZone.getX()+18,observationZone.getY()+botLength, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                //.addPath(new BezierLine(new Point(specimenMark21Control.getX(),specimenMark21Control.getY(), Point.CARTESIAN),
                //      new Point(observationZone.getX(),observationZone.getY()+botLength, Point.CARTESIAN)))
                //.setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        samplePick = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX()+18,observationZone.getY()+botLength, Point.CARTESIAN),
                        new Point(obsControl.getX(),obsControl.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
                .addPath(new BezierLine(new Point(obsControl.getX(),obsControl.getY(), Point.CARTESIAN),
                        new Point(observationZone.getX(),observationZone.getY()+botPickup, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(observationZone.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();

        sample1Place = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(observationZone.getX(),observationZone.getY()+botPickup, Point.CARTESIAN),
                        new Point(startPose.getX(), 0, Point.CARTESIAN),
                        new Point(highchamber.getX()-4, highchamber.getY()-24, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(observationZone.getHeading(),startPose.getHeading(),0.5)
                .addPath(new BezierLine(new Point(highchamber.getX()-4, highchamber.getY()-24, Point.CARTESIAN),
                        new Point(highchamber.getX()-4, highchamber.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        obs1Zone = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highchamber.getX()-4, highchamber.getY(), Point.CARTESIAN),
                        new Point(observationZone.getX(),observationZone.getY()+botPickup, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        sample2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY()+botPickup, Point.CARTESIAN),
                        new Point(highchamber.getX()-8, highchamber.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(observationZone.getHeading(),startPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(5)
                .build();

        obs2Zone = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highchamber.getX()-8, highchamber.getY(), Point.CARTESIAN),
                        new Point(observationZone.getX(),observationZone.getY()+botPickup, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
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
                    follower.followPath(fwdHighCmbr);

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
                if (pathTimer.getElapsedTime() > 500) {
                    robot.extArmPos(0, robot.extArmEPower);
                    robot.armPos(robot.armHighCaNew-150, robot.armEPower);

                    setPathState(10);
                }
                break;

            case 10: // yellow1 path
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(sample1,/* holdEnd = */ true);
                    // telemetry.addData("Status", "10");
                    setPathState(11);
                }

                    break;

            case 11: // Sample 1 push to obszone
                if (pathTimer.getElapsedTime() > 1500) {
                    robot.wallPick();
                    telemetry.addData("Status", "Sample1 push to observe");
                    setPathState(12);
                }

                break;

            case 12: // Sample 2 push
                if (follower.getPose().getY() < (observationZone.getY() + botLength) || pathTimer.getElapsedTime() > 4000) { //follower.getPose().getY() > (observationZone.getY() + botLength)
                    follower.followPath(sample2,/* holdEnd = */ true);

                    setPathState(13);
                }

                break;

            case 13: // Kraken turnaround
                if (follower.atParametricEnd() || pathTimer.getElapsedTime() > 3000) {

                    follower.followPath(samplePick,/* holdEnd = */ true);

                    setPathState(14);
                }
                break;

            case 14: // Specimen 1 grab and chamber placement
                if (follower.getPose().getY() < (observationZone.getY() + botPickup + 1) && pathTimer.getElapsedTime() > 1000) { //follower.getPose().getY() > (observationZone.getY() + botPickup + 0.5)
                    robot.closeClaw();

                    setPathState(141);
                }
                break;

            case 141: // To Chamber placement
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(sample1Place,/* holdEnd = */ true);

                    setPathState(142);
                }
                break;

            case 142: // Arm to Chamber placement
                if (pathTimer.getElapsedTime() > 250) {
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
                    follower.followPath(obs1Zone,/* holdEnd = */ true);

                    robot.wallPick();

                    setPathState(16);
                }
                break;

            default:
                break;
        }

/*
            case 16: // Specimen 2 grab and chamber placement
                if (pathTimer.getElapsedTime() > 250) {
                    robot.closeClaw();

                    if (pathTimer.getElapsedTime() > 500) {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristHighCw();
                        follower.followPath(samplePlace);
                    }
                    if (atPathEnd() && armSetUp() && pathTimer.getElapsedTime() > 2000) {
                        setPathState(17);
                    }
                }
                break;

            case 17: // Specimen 2 place
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.armPos(robot.armHighCa - 300, 0.4);
                    if (pathTimer.getElapsedTime() > 1350) {
                        robot.openClaw();
                    }

                    if (pathTimer.getElapsedTime() > 1450) {
                        follower.followPath(obsZone);
                    }

                    if (atPathEnd() && pathTimer.getElapsedTime() > 2500) {
                        setPathState(18);
                    }
                }

                break;

            case 18: //obszone and collapse
                if (pathTimer.getElapsedTime() > 50) {
                    robot.collapse(); }
                break;

            default:
                requestOpModeStop();
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
