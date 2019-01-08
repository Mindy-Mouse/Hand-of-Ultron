package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous
public class craterSide extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor             LeftFrontWheels = null;
    DcMotor             LeftBackWheels = null;
    DcMotor             RightFrontWheels = null;
    DcMotor             RightBackWheels = null;
    DcMotor             outLift = null;
    DcMotor             inLift = null;
    
    BNO055IMU           imu;
    ColorSensor         color1;
    ColorSensor         color2;
    ColorSensor         color3;
    Servo               marker;
    //motor.setMode(DcMotorController.runMode.RUN_WITHOUT_ENCODERS);
    //https://www.youtube.com/watch?annotation_id=annotation_3563505603&feature=iv&src_vid=d0liBxZCtrA&v=kOapppDNlSA
    //for autonomous with encoders ^^
    //lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //frontLeftW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //For teleop if you're having troubles  with motors, they really help make it easier to control the robot, do this for all motors ^^

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    Orientation lastAngles = new Orientation();

    ElapsedTime runtime = new ElapsedTime();
    // static final double     FORWARD_SPEED = 0.5;
    // static final double     TURN_SPEED    = 0.35;
    double globalAngle, power = DRIVE_SPEED, correction;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsv1[] = {0F, 0F, 0F};
    float hsv2[] = {0F, 0F, 0F};
    float hsv3[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    //final float values3[] = hsv3;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
   // int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
   // final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

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
        outLift = hardwareMap.get(DcMotor.class, "outLift");
        inLift = hardwareMap.get(DcMotor.class, "inLift");


        LeftFrontWheels.setDirection(DcMotor.Direction.REVERSE);
        LeftBackWheels.setDirection(DcMotor.Direction.REVERSE);
        RightFrontWheels.setDirection(DcMotor.Direction.FORWARD);
        RightBackWheels.setDirection(DcMotor.Direction.FORWARD);
        outLift.setDirection(DcMotor.Direction.REVERSE);

        color1 = hardwareMap.colorSensor.get("color1");
        color2 = hardwareMap.colorSensor.get("color2");
        color3 = hardwareMap.colorSensor.get("color3");

        marker = hardwareMap.servo.get("marker");

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
        outLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        
        outLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outLift.setPower(0);
        telemetry.addData("Cascade encoder: ", outLift.getCurrentPosition());
        
        waitForStart();

        //*******************************************************************************************************************************************************************
        outLift.setTargetPosition(3900);    //land
        outLift.setPower(0.9);
        gyroDrive(DRIVE_SPEED,6.0);    // Drive forward 6 inches
        outLift.setTargetPosition(0);    //bring hook down
        outLift.setPower(0.5);
        rotate( TURN_SPEED,  180);         // Turn  right  to  180 Degrees
        gyroDrive(DRIVE_SPEED,-6.0);    // Drive backward 6 inches

        Color.RGBToHSV((int) (color1.red() * SCALE_FACTOR), (int) (color1.green() * SCALE_FACTOR),(int) (color1.blue() * SCALE_FACTOR), hsv1);
        Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR), (int) (color2.green() * SCALE_FACTOR),(int) (color2.blue() * SCALE_FACTOR), hsv2);
        Color.RGBToHSV((int) (color3.red() * SCALE_FACTOR), (int) (color3.green() * SCALE_FACTOR),(int) (color3.blue() * SCALE_FACTOR), hsv3);

        int     position = 0;

        for (int i = 0; i < 3; i++) {
            if ((hsv1[0] < 100)|| (hsv2[0] < 100)|| (hsv3[0] < 100)){
                //move sampling mechanism
                i = 5;
                sleep(5000);
            }
            else if ((hsv1[0] > 100)|| (hsv2[0] > 100)|| (hsv3[0] > 100)){
                if(position == 0){
                    gyroDrive(DRIVE_SPEED, 5.0);    // Drive forward 5 inches
                    rotate(TURN_SPEED, -90);    //Turn left 90
                    gyroDrive(DRIVE_SPEED, 14.5);   //Drive forward 14.5 inches
                    rotate(TURN_SPEED, 90);     //Turn right 90
                    gyroDrive(DRIVE_SPEED, -5.0);   //Drive backward 5 inches to mineral
                }

                if(position == 1){
                    gyroDrive(DRIVE_SPEED, 5.0);    // Drive forward 5 inches
                    rotate(TURN_SPEED, 90);    //Turn right 90
                    gyroDrive(DRIVE_SPEED, 30);   //Drive forward 30 inches
                    rotate(TURN_SPEED, -90);     //Turn left 90
                    gyroDrive(DRIVE_SPEED, -5.0);   //Drive backward 5 inches to mineral
                }

            }
            position++;
        }

        gyroDrive(DRIVE_SPEED, 5.0);    //move mineral
        gyroDrive(DRIVE_SPEED, 6.0);   // drives away

        rotate(TURN_SPEED,  -90);         // Turn left 90 Degrees

        if (position == 1){     //gold in middle
            gyroDrive(DRIVE_SPEED,-26);      // Drive backward 26 inches move along lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            rotate(TURN_SPEED,  -90);         // Turn left 90 degrees turn parallel to the other side of lander
            gyroDrive(DRIVE_SPEED,-36);    // Drive backward 36 inches move along the lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            rotate(TURN_SPEED, 92);     //turn right 92 degrees
            gyroDrive(DRIVE_SPEED, -40);    //back up into depot

            marker.setPosition(0.3);    //drop marker
            marker.setPosition(0.7);    //retratct
            telemetry.addLine("Marker dropped");
            
            gyroDrive(DRIVE_SPEED, 40);    //drive away from depot
            rotate(TURN_SPEED, -92);     //turn left 92 degrees
            gyroDrive(DRIVE_SPEED, 36);    // Drive forward 36 inches move along the lander line
            rotate(TURN_SPEED,  90);         // Turn right 90 degrees turn parallel to the other side of lander
            gyroDrive(DRIVE_SPEED, 36);      // Drive forward 36 inches move along lander line
            
            rotate(TURN_SPEED,  -90);         // Turn left 90 degrees towards minerals
            gyroDrive(DRIVE_SPEED, 5.0);    //move towards minerals
            
            inLift.setPower(TURN_SPEED);    //extend front basket into crater
            sleep(3000);
            inLift.setPower(0);
        }
        
        else if (position == 2){     //gold on right
            gyroDrive(DRIVE_SPEED,-41);      // Drive backward 41 inches move along lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            rotate(TURN_SPEED,  -90);         // Turn left 90 degrees turn parallel to the other side of lander
            gyroDrive(DRIVE_SPEED,-26);    // Drive backward 26 inches move along the lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            rotate(TURN_SPEED, 85);     //turn right 85 degrees
            gyroDrive(DRIVE_SPEED, -45);    //back up into depot

            marker.setPosition(0.3);    //drop marker
            marker.setPosition(0.7);    //retratct
            telemetry.addLine("Marker dropped");
            
            gyroDrive(DRIVE_SPEED, 45);    //drive away from depot
            rotate(TURN_SPEED, -85);     //turn left 85 degrees
            gyroDrive(DRIVE_SPEED, 36);    // Drive forward 36 inches move along the lander line
            rotate(TURN_SPEED,  90);         // Turn right 90 degrees turn parallel to the other side of lander
            gyroDrive(DRIVE_SPEED, 40);      // Drive forward 40 inches move along lander line
            
            rotate(TURN_SPEED,  -90);         // Turn left 90 degrees towards minerals
            gyroDrive(DRIVE_SPEED, 5.0);    //move towards minerals
            
            inLift.setPower(TURN_SPEED);    //extend front basket into crater
            sleep(3000);
            inLift.setPower(0);
        }
        
        else if (position == 3){     //gold on left
            gyroDrive(DRIVE_SPEED,-11);      // Drive backward 11 inches move along lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            rotate(TURN_SPEED,  -90);         // Turn left 90 degrees turn parallel to the other side of lander
            gyroDrive(DRIVE_SPEED,-48);    // Drive backward 48 inches move along the lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            rotate(TURN_SPEED, 97);     //turn right 97 degrees
            gyroDrive(DRIVE_SPEED, -46);    //back up into depot

            marker.setPosition(0.3);    //drop marker
            marker.setPosition(0.7);    //retratct
            telemetry.addLine("Marker dropped");
            
            gyroDrive(DRIVE_SPEED, 46);    //drive away from depot
            rotate(TURN_SPEED, -97);     //turn left 92 degrees
            gyroDrive(DRIVE_SPEED, 48);    // Drive forward 48 inches move along the lander line
            rotate(TURN_SPEED,  90);         // Turn right 90 degrees turn parallel to the other side of lander
            gyroDrive(DRIVE_SPEED, 48);      // Drive forward 48 inches move along lander line
            
            rotate(TURN_SPEED,  -90);         // Turn left 90 degrees towards minerals
            gyroDrive(DRIVE_SPEED, 5.0);    //move towards minerals
            
            inLift.setPower(TURN_SPEED);    //extend front basket into crater
            sleep(3000);
            inLift.setPower(0);
        }

      /*  rotate( TURN_SPEED,  45);          // Turn right 45 degrees          turn perpendicular to the wall
        gyroDrive(DRIVE_SPEED,-36);      // Drive backward 36 inches      move to the wall
        telemetry.addData("Distance: ", imu.getVelocity());
        telemetry.update();
        rotate( TURN_SPEED,  90);          // Turn right 90 degrees          turn to the depot
        gyroDrive(DRIVE_SPEED,-50.2);     // Drive backward 50.2 inches   move to the depot
        rotate( TURN_SPEED,  45);         // Turn  right  to  45 Degrees
        gyroDrive(DRIVE_SPEED,5.0);    // Drive forward 5 inches
*/

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
    public void rotate(double power, int degrees) {
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


