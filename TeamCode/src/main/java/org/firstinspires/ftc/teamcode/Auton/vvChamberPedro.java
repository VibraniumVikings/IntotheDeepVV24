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

    private PathChain sample1, sample2Drop, samplePlace, obsZone;

    private int pathState;

    // We want to start the bot at x: 14, y: -60, heading: 90 degrees
    private Pose startPose = new Pose(79, 7, Math.toRadians(90));
    // all sample mark locations
    private Pose highchamber = new Pose(79,33.5);
    private Pose sampleMark1 = new Pose(32.5,27);
    private Pose sampleMark2 = new Pose(22.5,27);
    private Pose sampleMark3 = new Pose(12.5,27,Math.toRadians(180));
    private Pose dropposition = new Pose (16,16,Math.toRadians(45));
    private Pose specimenMark1 = new Pose(36+72, -45+72);
    private Pose specimenMark2 = new Pose(24.5+72, -45+72);
    private Pose specimenMark3 = new Pose(36+72, -45+72,Math.toRadians(0));
    private Pose observationZone= new Pose(132,16, Math.toRadians(270));
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
    public boolean isAtEndOfPathAndNotMoving() {
        return atPathEnd() && follower.getVelocityMagnitude() < 0.01;
    }
    public boolean armSetUp() {
        return robot.arm.getCurrentPosition()>robot.arm.getTargetPosition()-10;
    }
    public boolean armSetDown() {
        return robot.arm.getCurrentPosition()>robot.arm.getTargetPosition()+10;
    }

    public void buildPaths() {

        fwdHighCmbr = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN))); //Tile Start Position
        fwdHighCmbr.setConstantHeadingInterpolation(startPose.getHeading());
        fwdHighCmbr.setPathEndTimeoutConstraint(2);

        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN), new Point(highchamber.getX()+25, highchamber.getY()-15, Point.CARTESIAN), new Point(specimenMark1.getX()-3, specimenMark1.getY()+17, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(specimenMark1.getX()-3, specimenMark1.getY()+17, Point.CARTESIAN), new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN), new Point(specimenMark2.getX(),specimenMark2.getY()-botPickup, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();

        sample2Drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenMark2.getX(),specimenMark2.getY()-botPickup, Point.CARTESIAN), new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        samplePlace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN), new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(observationZone.getHeading(),startPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        obsZone = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN), new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();
    }

        public void autonPathUpdate() {
        switch (pathState){
            case 9: //move arm to position
                robot.rgb.setPosition(0.5);
                robot.armPos(robot.armHighCa, robot.armEPower);
                //robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                robot.moveWristHighCw();

                if (robot.arm.getCurrentPosition() > robot.arm.getTargetPosition() - 900) {
                    setPathState(10);
                }
                break;

            case 10: //high chamber specimen placement
                follower.followPath(fwdHighCmbr);

                if (isAtEndOfPathAndNotMoving()) {
                    setPathState(11);
                }

                break;

            case 11: // Sample clip and sample 1 push to obszone
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.armPos(robot.armHighCa - 300, 0.4);
                    if (pathTimer.getElapsedTime() > 1350) {
                        robot.openClaw();
                    }

                    //robot.extArmPos(50, robot.extArmEPower);
                    if (pathTimer.getElapsedTime() > 1450) {
                        follower.followPath(sample1);
                    }

                    if (atPathEnd()) {
                        setPathState(12);
                    }
                }

                break;

            case 12: // Sample 2 grab
                if (pathTimer.getElapsedTime() > 100) {
                    robot.pickSample();

                    if (pathTimer.getElapsedTime() > 300) { // && pathTimer.getElapsedTime() > 2000
                        robot.closeClaw();

                        setPathState(13);
                    }
                }

                break;

            case 13: // Sample 2 drop
                if (pathTimer.getElapsedTime() > 250) {
                    robot.wallPick();

                    follower.followPath(sample2Drop);

                    if (isAtEndOfPathAndNotMoving() && armSetUp()) {
                        robot.openClaw();
                        setPathState(14);
                    }
                }
                break;

            case 14: // Specimen 1 grab and chamber placement
                if (pathTimer.getElapsedTime() > 250) {
                    robot.closeClaw();

                    if (pathTimer.getElapsedTime() > 500) {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristHighCw();
                        follower.followPath(samplePlace);
                    }
                    if (isAtEndOfPathAndNotMoving() && armSetUp()) {
                        setPathState(15);
                    }
                }
                break;

            case 15: // Specimen 1 place
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.armPos(robot.armHighCa - 300, 0.4);
                    if (pathTimer.getElapsedTime() > 1350) {
                        robot.openClaw();
                    }

                    if (pathTimer.getElapsedTime() > 1450) {
                        follower.followPath(obsZone);
                    }
                    if (pathTimer.getElapsedTime() > 2000) {
                        robot.wallPick(); }

                    if (isAtEndOfPathAndNotMoving()) {
                        setPathState(16);
                    }
                }

                break;

            case 16: // Specimen 2 grab and chamber placement
                if (pathTimer.getElapsedTime() > 250) {
                    robot.closeClaw();

                    if (pathTimer.getElapsedTime() > 500) {
                        robot.armPos(robot.armHighCa, robot.armEPower);
                        robot.moveWristHighCw();
                        follower.followPath(samplePlace);
                    }
                    if (isAtEndOfPathAndNotMoving() && armSetUp()) {
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

                    if (atPathEnd()) {
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
            if (opmodeTimer.getElapsedTimeSeconds() > 28) {
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
