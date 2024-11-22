package org.firstinspires.ftc.teamcode.Core;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** This class is nearly identical to vvHardwareITD, except there are differences due to the auton starting position difference */

public class vvHardwareITDRR {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;
    public DcMotorEx leftRear;
    public DcMotorEx arm;
    public DcMotorEx extend;
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public Servo wrist;
    public Servo claw;
    public Servo rgb;
    public Servo led;

    public IMU imu;
    public DcMotor parallelEncoder;
    public DcMotor perpendicularEncoder;
    //public ColorSensor colorSensor;
    //public DistanceSensor distFront;
    //public DistanceSensor distRear;
    private ElapsedTime runtime = new ElapsedTime();

    /* Define Drive constants.  Make them public so they CAN be used by the calling OpMode
     * arm variables: floorArm, highCa, lowCa, highBa, lowBa
     * extension variables: floorTuck, full, highCe, lowCe, highBe, lowBe
     * wrist variables: floorPick, highCw, lowCw, highBw, lowBw
     * claw variables: openClaw, closeClaw (Do we need to add one for length vs. width samples?)
     */
    public static final double clawClose      =  1 ; //Metal 1,3D Print 0.9
    public static final double clawLong     =  0.8 ;
    public static final double clawOpen       =  0.65 ; //Metal 0.65, 3D Print 0.3
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = 0.45 ;
    public static final double floorPick = 0.1 ;
    public static final double floorCarry = 0.9 ;
    public static final double highCw  = 0.3 ;
    public static final double lowCW = 0.5 ;
    public static final double highBw = 0.2 ;
    public static final double lowBw = 0.3 ;
    public static final double lowWallCw = 0.4 ;

    final public int floorArm = 0;// -84
    final public int armLowCa = 550; //
    final public int armHighCa = 1250; //was 1200
    final public int armLowBa = 1450;
    final public int armHighBa = 2159;
    final public int armRearBa = 2600;
    final public int armFloorSub = 400;
    final public int armWall = 350; //400
    final public int armAscent = 1500;
    final public double armEPower = 0.5;
    final public int extArmHighBe = 2000;
    final public int extArmLowBe = 838;
    final public int extArmHighCe = 600;
    final public int extArmLowCe = 50;
    final public int extArmFloorSub= 1450;
    final public int extArmFLoorPick = 290;
    final public double extArmEPower = 0.4;

    final public double liftEPower = 0.5;
    final public int liftHigh = 2000; //Max is 2200, 24"-32" height, 20"mid is 1100
    final public int liftLow = 50;

    static final double FORWARD_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    final public int hcv = 20;
    final public int hca = 20;
    final public int hspdv = 60;
    final public int hspda = 60;
    public static double WHEEL_DIAMETER = 1.88976; // in
    public static final double TICKS_PER_REV = 2000;
    public double encTicksPerInches = TICKS_PER_REV/(WHEEL_DIAMETER*Math.PI);
    public double encInchesPerTicks = (WHEEL_DIAMETER*Math.PI)/TICKS_PER_REV;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public vvHardwareITDRR(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        extend = myOpMode.hardwareMap.get(DcMotorEx.class, "extend");
        arm = myOpMode.hardwareMap.get(DcMotorEx.class, "arm");
        leftLift = myOpMode.hardwareMap.get(DcMotorEx.class, "ltLift");
        rightLift = myOpMode.hardwareMap.get(DcMotorEx.class, "rtLift");

        //Shadow the motors with encoder-odometry
        //parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        //perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        perpendicularEncoder = leftFront;
        parallelEncoder = rightFront; //Will need to use an opposite sign for right

        // Define Servos
        claw = myOpMode.hardwareMap.get(Servo.class,"claw");
        wrist = myOpMode.hardwareMap.get(Servo.class,"wrist");
        rgb = myOpMode.hardwareMap.get(Servo.class,"rgb");
        led = myOpMode.hardwareMap.get(Servo.class, "led");

        wrist.scaleRange(0,1);
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(floorCarry);

        claw.scaleRange(0,1);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(clawClose);

        rgb.scaleRange(0,1);
        rgb.setDirection(Servo.Direction.FORWARD);
        rgb.setPosition(0.7);

        led.scaleRange(0,1);
        led.setDirection(Servo.Direction.FORWARD);
        led.setPosition(0);

        //Set the motor directions

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("Claw", claw.getPosition());
        myOpMode.telemetry.update();
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param armPower driving power (-1.0 to 1.0)
     */
    public void moveArm(double armPower) {
        arm.setPower(armPower);
    }
    public void moveExt(double extPower) {
        extend.setPower(extPower);
    }
    /*
     * Pass the requested arm position and power to the arm drive motors
     *
     * @param armEPower driving power (-1.0 to 1.0)
     * @param armPosition full lift range is XXXX to XXXX
     * @param , extension driving power
     * @param  full lift range is XXXX to XXXX
     */
    public void armPos(int armPosition, double armEPower) {
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armEPower);
    }
    public void extArmPos(int extArmPosition, double extArmEPower) {
        extend.setTargetPosition(extArmPosition);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setPower(extArmEPower);
    }
    public void liftUp() {
        leftLift.setTargetPosition(liftHigh);
        rightLift.setTargetPosition(liftHigh);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(liftEPower);
        rightLift.setPower(liftEPower);
    }
    public void liftDown() {
        leftLift.setTargetPosition(liftLow);
        rightLift.setTargetPosition(liftLow);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(liftEPower);
        rightLift.setPower(liftEPower);
    }
    public void moveWristFloor() {
        wrist.setPosition(floorPick);
    }
    public void moveWristCarry() {
        wrist.setPosition(floorCarry);
    }
    public void moveWristHighCw() {
        wrist.setPosition(highCw);
    }
    public void moveWristLowCW() {
        wrist.setPosition(lowCW);
    }
    public void moveWristLowBw() {
        wrist.setPosition(lowBw);
    }
    public void moveWristHighBw() {
        wrist.setPosition(highBw);
    }
    public void moveWristWall() {wrist.setPosition(lowWallCw);}
    /**
     * Set the claw servo to close
     *
     * @param
     */
    public void openClaw() {
        claw.setPosition(clawOpen);
        rgb.setPosition(0.29);
        led.setPosition(0.8);
    }
    public void closeClaw() {
        claw.setPosition(clawClose);
        rgb.setPosition(0.5);
        led.setPosition(0);
    }

}

