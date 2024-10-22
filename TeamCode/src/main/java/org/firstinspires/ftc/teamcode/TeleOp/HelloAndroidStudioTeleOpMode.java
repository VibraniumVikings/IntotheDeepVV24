package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="VF TeleOp - Coach", group="1")
public class HelloAndroidStudioTeleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft = null;

    private DcMotor backRight = null;

    private DcMotor frontRight = null;

    private DcMotor frontLeft = null;

    // start at 50% power
    private double powerFactor = 0.5;

    /* multiplication factor used to control acceleration - a higher factor means faster
     * acceleration, a lower number means slower acceleration.  A factor higher than .5 accelerates
     * so fast that we have only two effective speeds because of the speed the loop runs in
     */
    public static final double ACCELERATION_FACTOR = 0.003;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeft = hardwareMap.get(DcMotor.class, "RLM");
        backRight = hardwareMap.get(DcMotor.class, "RRM");
        frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        frontRight = hardwareMap.get(DcMotor.class, "FRM");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // because we have four motors we need set all 4 motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection((DcMotorSimple.Direction.REVERSE));
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;


            // set up the left bumper button to decelerate and right bumper to accelerate
            if (gamepad1.left_bumper) {
                powerFactor = Math.max(powerFactor - ACCELERATION_FACTOR, 0.1);
            }
            if (gamepad1.right_bumper) {
                powerFactor = Math.min(powerFactor + ACCELERATION_FACTOR, 1.0);
            }

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;


            frontLeftPower   = powerFactor * (Range.clip(drive + turn, -1.0, 1.0)) ;
            frontRightPower  = powerFactor * (Range.clip(drive - turn, -1.0, 1.0)) ;
            backLeftPower    = powerFactor * (Range.clip(drive + turn, -1.0, 1.0)) ;
            backRightPower   = powerFactor * (Range.clip(drive - turn, -1.0, 1.0)) ;

            // Send calculated power to wheels
            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)"
                    , frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}

