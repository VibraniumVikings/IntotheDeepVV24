package org.firstinspires.ftc.teamcode.Core;

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
import org.firstinspires.ftc.teamcode.util.Encoder;

/*
 * New hardware class for the ITD (Into The Deep) game
 * Need to add single arm motor, extension motor, wrist, and claw
 */


public class vvHardwareITD {


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
    public static final double clawClose      =  1 ; //Metal 13D Print 0.9
    public static final double clawLong     =  0.8 ; //Metal 0.8 3D Print 0.5
    public static final double clawOpen       =  0.65 ; //Metal 0.65 3D Print 0.3
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = 0.45 ;
    public static final double floorPick = 0.1 ; //Metal 0.1 3D Print 0.4
    public static final double floorCarry = 0.9 ;
    public static final double highCw  = 0.3 ;
    public static final double lowCW = 0.5 ;
    public static final double highBw = 0.2 ;
    public static final double lowBw = 0.3 ;
    public static final double lowWallCw = 0.4 ;

    final public int floorArm = 0;// 0 for metal, ~50 for claw?
    final public int armLowCa = 550; //
    final public int armHighCa = 1200; //
    final public int armLowBa = 1450;
    final public int armHighBa = 2159;
    final public int armRearBa = 2600; //Max arm is 3747, 2560 is vertical
    final public int armFloorSub = 400; // 400 for metal, ~200 for claw?
    final public int armWall = 400;
    final public int armAscent = 1500;
    final public double armEPower = 0.5;
    final public int extArmAscentGrab = 1450; //20" high is 1450
    final public int extArmAscentLift = 50;
    final public int extArmHighBe = 2000; //Max is 2035, 36" to 47" reach, 13.5" fulcrum
    final public int extArmLowBe = 838;
    final public int extArmHighCe = 600;
    final public int extArmLowCe = 50;
    final public int extArmFloorSub= 1450;
    final public int extArmFLoorPick = 290; //Metal 290, 3D Print 400
    final public double extArmEPower = 0.4;

    final public double liftEPower = 0.5;
    final public int liftHigh = 2000; //Max is 2200, 24"-32" height, 20"mid is 1100
    final public int liftLow = 50;

    static final double FORWARD_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    public static double WHEEL_DIAMETER = 1.88976; // in
    public static final double TICKS_PER_REV = 2000;
    public double encTicksPerInches = TICKS_PER_REV/(WHEEL_DIAMETER*Math.PI);
    public double encInchesPerTicks = (WHEEL_DIAMETER*Math.PI)/TICKS_PER_REV;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public vvHardwareITD(LinearOpMode opmode) {
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
        leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "FLM");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "FRM");
        rightRear = myOpMode.hardwareMap.get(DcMotorEx.class, "RRM");
        leftRear = myOpMode.hardwareMap.get(DcMotorEx.class, "RLM");
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

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //imu.resetYaw(); //reset the imu during initialization

        //Set the motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("Claw", claw.getPosition());
        myOpMode.telemetry.update();
    }
    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param drivePower Global variable to set total drive power
     * @param driveY    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     *                  ..
     * @param strafe    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double drivePower, double driveY, double strafe, double turn) {
        double strafeBias = 1; // given weight distribution need to bias the back wheel power
        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe*strafeBias) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - (strafe*strafeBias) + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + (strafe*strafeBias) - turn) / denominator;

        leftFront.setPower(drivePower * frontLeftPower);
        leftRear.setPower(drivePower * backLeftPower);
        rightFront.setPower(drivePower * frontRightPower);
        rightRear.setPower(drivePower * backRightPower);
    }
    /**
     * Field centric method
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param drivePower Global variable to set total drive power
     * @param driveYfc    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafeFC    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobotFC(double drivePower, double driveYfc, double strafeFC, double turn) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double driveY = driveYfc * Math.cos(botHeading) - strafeFC * Math.sin(botHeading);
        double strafe = driveYfc * Math.sin(botHeading) + strafeFC * Math.cos(botHeading);

        double strafeBias = 1; // given weight distribution need to bias the back wheel power
        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe*strafeBias) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - (strafe*strafeBias) + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + (strafe*strafeBias) - turn) / denominator;

        leftFront.setPower(drivePower * frontLeftPower);
        leftRear.setPower(drivePower * backLeftPower);
        rightFront.setPower(drivePower * frontRightPower);
        rightRear.setPower(drivePower * backRightPower);
    }
    /**
     * Sets the target for the chassis to achieve the desired position and heading
     * robot motions: Drive (Axial-X motion), Strafe (Side-to-Side-Y motion) and Turn (Yaw-Z motion).
     * Then sends these power levels to the motors.
     * @param autonPower Global variable to set total drive power
     * @param driveY    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Side to side driving power (-1.0 to 1.0) +ve is left
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveAuton(double autonPower, double driveY, double strafe, double turn) {
        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (driveY + strafe + turn) / denominator;
        double backLeftPower = (driveY - strafe + turn) / denominator;
        double frontRightPower = (driveY - strafe - turn) / denominator;
        double backRightPower = (driveY + strafe - turn) / denominator;

        leftFront.setPower(autonPower * frontLeftPower);
        leftRear.setPower(autonPower * backLeftPower);
        rightFront.setPower(autonPower * frontRightPower);
        rightRear.setPower(autonPower * backRightPower);
    }
    public void driveForward(double dTime, double distance) {
        driveRobot(1, FORWARD_SPEED, 0, 0);

        runtime.reset();
        while (myOpMode.opModeIsActive() && ((runtime.seconds() < dTime)||(-parallelEncoder.getCurrentPosition()>(distance*encTicksPerInches))));
    }
    public void driveBackward (double dTime, double distance) {
        driveRobot(1, -FORWARD_SPEED, 0, 0);

        runtime.reset();
        while (myOpMode.opModeIsActive() && ((runtime.seconds() < dTime)||(-parallelEncoder.getCurrentPosition()<(distance*encTicksPerInches))));
        driveRobot(0, 0, 0, 0);
    }
    public void driveRight(double dTime, double distance) {
        driveRobot(1, 0, FORWARD_SPEED, 0);

        runtime.reset();
        while (myOpMode.opModeIsActive() && ((runtime.seconds() < dTime)||(-perpendicularEncoder.getCurrentPosition()>(distance*encTicksPerInches))));
        driveRobot(0, 0, 0, 0);
    }
    public void driveLeft(double dTime, double distance) {
        driveRobot(1, 0, -FORWARD_SPEED, 0);

        runtime.reset();
        while (myOpMode.opModeIsActive() && ((runtime.seconds() < dTime)||(-perpendicularEncoder.getCurrentPosition()<(distance*encTicksPerInches))));
        driveRobot(0, 0, 0, 0);
    }
    public void turnRight(double dTime, double desHeading) {
        driveRobot(1, 0, 0, TURN_SPEED);

        runtime.reset();
        while (myOpMode.opModeIsActive() && ((runtime.seconds() < dTime)||(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > desHeading)));
        driveRobot(0, 0, 0, 0);
    }
    public void turnLeft(double dTime, double desHeading) {
        driveRobot(1, 0, 0, -TURN_SPEED);

        runtime.reset();
        while (myOpMode.opModeIsActive() && ((runtime.seconds() < dTime)||(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -desHeading)));
        driveRobot(0, 0, 0, 0);
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
    public void moveLift(double liftPower) {
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
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
    public void longClaw() {
        claw.setPosition(clawLong);
        rgb.setPosition(0.388);
    }

}

