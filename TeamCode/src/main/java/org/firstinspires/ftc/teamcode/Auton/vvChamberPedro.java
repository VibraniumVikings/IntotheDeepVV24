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

    private PathChain sample1, sample2, sample2Drop, samplePlace, obsZone;

    private int pathState;

    // We want to start the bot at x: 14, y: -60, heading: 90 degrees
    private Pose startPose = new Pose(79, 7, Math.toRadians(90));
    // all sample mark locations
    private Pose highchamber = new Pose(79,33.5);
    private Pose sampleMark1 = new Pose(22.5,43);
    private Pose sampleMark2 = new Pose(12.5,43);
    private Pose sampleMark3 = new Pose(2.5,43,Math.toRadians(180));
    private Pose dropposition = new Pose (16,16,Math.toRadians(45));
    private Pose specimenMark1 = new Pose(121.5, 43);
    private Pose specimenMark2 = new Pose(131.5, 43);
    private Pose specimenMark3 = new Pose(141.5, 43,Math.toRadians(0));
    private Pose observationZone= new Pose(132,16, Math.toRadians(-90));
    //Kraken dimensional offsets
    public double botWidth = 7;
    public double botLength = 7;
    public double botPickup = 11;

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
                                         new Point(108, 16, Point.CARTESIAN), 
                                         new Point(specimenMark1.getX()-9, specimenMark1.getY()+15, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(specimenMark1.getX()-9, specimenMark1.getY()+15, Point.CARTESIAN), 
                                        new Point(observationZone.getX(),observationZone.getY()-botLength, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(5)
                .build();

        sample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY()-botLength, Point.CARTESIAN), 
                                        new Point(specimenMark2.getX(),specimenMark2.getY()-botPickup, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();

        sample2Drop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenMark2.getX(),specimenMark2.getY()-botPickup, Point.CARTESIAN), 
                                        new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();

        samplePlace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN), 
                                        new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(observationZone.getHeading(),startPose.getHeading(),0.5)
                .setPathEndTimeoutConstraint(5)
                .build();

        obsZone = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highchamber.getX(), highchamber.getY(), Point.CARTESIAN), 
                                        new Point(observationZone.getX(),observationZone.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),observationZone.getHeading(),0.5)
                .setPathEndTimeoutConstraint(2)
                .build();
    }

        public void autonPathUpdate() {
        switch (pathState){
            case 6: //move arm to position and high chamber specimen placment
                if (follower.getPose().getY() > (startPose.getY() - 1)) {
                    robot.rgb.setPosition(0.5);
                    robot.armPos(robot.armHighCaNew, robot.armEPower);
                    robot.moveWristHighCwNew();
                    robot.extArmPos(robot.extArmHighCe, robot.extArmEPower);
                    setPathState(7);
                }
                break;

            case 7: //high chamber path
                if (pathTimer.getElapsedTime() > 1500) {
                    follower.followPath(fwdHighCmbr);

                    setPathState(8);
                }
                break;

            case 8: // High chamber open claw
                if (follower.getPose().getX() > (highchamber.getX() - 1) && follower.getPose().getY() > (highchamber.getY() - 1)) {
                    robot.openClaw();
                    //robot.moveWristFloor();

                    setPathState(9);
                }

                break;
            case 9: // High chamber retract arm
                if (pathTimer.getElapsedTime() > 500) {
                    robot.extArmPos(0, robot.extArmEPower);

                    setPathState(10);
                }
                break;

            case 10: // sample1 path
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(sample1);

                    setPathState(11);
                }
                break;

            case 11: // Ssample 1 push to obszone
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.pickSample();
                    setPathState(12);
                }

                break;

            case 12: // Sample 2 grab
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(sample2);

                    if (atPathEnd() && pathTimer.getElapsedTime() > 1000) { // && pathTimer.getElapsedTime() > 2000
                        robot.closeClaw();
                        setPathState(13);
                    }
                }

                break;

            case 13: // Sample 2 drop
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.wallPick();

                    follower.followPath(sample2Drop);

                    if(isHalfwayThere()) {
                        robot.openClaw();
                    }

                    if (atPathEnd() && armSetUp() && pathTimer.getElapsedTime() > 2000) { //atPathEnd() &&

                        setPathState(14);
                    }
                }
                break;

            case 14: // Specimen 1 grab and chamber placement
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.closeClaw();

                    setPathState(141);
                }
                break;

            case 141: // To Chamber placement
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(samplePlace);

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
                if (follower.getPose().getX() > (highchamber.getX() - 1) && follower.getPose().getY() > (highchamber.getY() - 1)) {
                    robot.openClaw();

                    setPathState(144);
                }

                break;
            case 144: // High chamber retract arm
                if (pathTimer.getElapsedTime() > 500) {
                    robot.extArmPos(0, robot.extArmEPower);

                    setPathState(15);
                }
                break;

            case 15: // Back to observation zone
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(obsZone);
                    
                    robot.wallPick(); }

                    setPathState(16);
                }

                break;
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
