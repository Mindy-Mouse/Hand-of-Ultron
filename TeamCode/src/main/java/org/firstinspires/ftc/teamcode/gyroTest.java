package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="gyroTest", group="Autonomous")
public class gyroTest extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor             LeftFrontWheels = null;
    DcMotor             LeftBackWheels = null;
    DcMotor             RightFrontWheels = null;
    DcMotor             RightBackWheels = null;
    BNO055IMU           imu;
    ColorSensor         botColor;

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    Orientation         lastAngles = new Orientation();

    ElapsedTime             runtime = new ElapsedTime();
   // static final double     FORWARD_SPEED = 0.5;
   // static final double     TURN_SPEED    = 0.35;
    double globalAngle, power = DRIVE_SPEED, correction;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        LeftFrontWheels = hardwareMap.get(DcMotor.class,"LFW");
        RightFrontWheels = hardwareMap.get(DcMotor.class,"RFW");
        RightBackWheels = hardwareMap.get(DcMotor.class,"RBW");
        LeftBackWheels = hardwareMap.get(DcMotor.class,"LBW");

        LeftFrontWheels.setDirection(DcMotor.Direction.REVERSE);
        LeftBackWheels.setDirection(DcMotor.Direction.REVERSE);
        RightFrontWheels.setDirection(DcMotor.Direction.FORWARD);
        RightBackWheels.setDirection(DcMotor.Direction.FORWARD);

        botColor = hardwareMap.colorSensor.get("botColor");

        // sets IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        LeftFrontWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        // Send telemetry message to alert driver that we are calibrating;
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        LeftFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 29.0);    // Drive FWD 48 inches
        rotate( TURN_SPEED, -45);         // Turn  left to -45 Degrees
        gyroDrive(DRIVE_SPEED, -15.0);  // Drive back 15 inches

        if (botColor.hashCode() < 15){
            //move sampling mechanism
            rotate( TURN_SPEED,  45);         // Turn  right  to  45 Degrees
            gyroDrive(DRIVE_SPEED,5.0);    // Drive REV 48 inches
        }

        else if (botColor.hashCode() > 15){
            gyroDrive(DRIVE_SPEED, 15.0);    // Drive FWD 15 inches
        }

       /* rotate( TURN_SPEED,  45);         // Turn  right  to  45 Degrees
      //  gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        //rotate( TURN_SPEED,   0);         // Turn  CW  to   0 Degrees
        //gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,5.0);    // Drive REV 48 inches*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     *
     */
    public void gyroDrive ( double speed, double distance) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = LeftFrontWheels.getCurrentPosition() + moveCounts;
            newRightTarget = RightFrontWheels.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            LeftFrontWheels.setTargetPosition(newLeftTarget);
            RightFrontWheels.setTargetPosition(newRightTarget);

            LeftFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // LeftBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // RightBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            RightFrontWheels.setPower(speed);
            RightBackWheels.setPower(speed);
            LeftFrontWheels.setPower(speed);
            LeftBackWheels.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (LeftFrontWheels.isBusy() && RightFrontWheels.isBusy())) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();

                LeftFrontWheels.setPower(-speed + correction);
                LeftBackWheels.setPower(-speed + correction);
                RightFrontWheels.setPower(-speed);
                RightBackWheels.setPower(-speed);


            }

            // Stop all motion;
            RightFrontWheels.setPower(0);
            RightBackWheels.setPower(0);
            LeftFrontWheels.setPower(0);
            LeftBackWheels.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // LeftBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //RightBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
   /* public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            correction = checkDirection();

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            LeftFrontWheels.setPower(-power + correction);
            LeftBackWheels.setPower(-power + correction);
            RightFrontWheels.setPower(-power);
            RightBackWheels.setPower(-power);
            telemetry.update();
        }

        RightFrontWheels.setPower(0);
        RightBackWheels.setPower(0);
        LeftFrontWheels.setPower(0);
        LeftBackWheels.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   //targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double checkDirection() {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0) {
            correction = 0;
        }
        // no adjustment.
        else {
            correction = -angle;        // reverse sign of angle for correction.
        }
        correction = correction * gain;

        return correction;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void rotate(double power, int degrees) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        LeftFrontWheels.setPower(leftPower);
        LeftBackWheels.setPower(leftPower);
        RightFrontWheels.setPower(rightPower);
        RightBackWheels.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        RightFrontWheels.setPower(0);
        RightBackWheels.setPower(0);
        LeftFrontWheels.setPower(0);
        LeftBackWheels.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}


