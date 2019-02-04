package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous
public class craterSide extends LinearOpMode {

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
    DistanceSensor distance1;
    DistanceSensor distance2;
    DistanceSensor distance3;
    Servo               marker;
    //motor.setMode(DcMotorController.runMode.RUN_WITHOUT_ENCODERS);
    //https://www.youtube.com/watch?annotation_id=annotation_3563505603&feature=iv&src_vid=d0liBxZCtrA&v=kOapppDNlSA
    //for autonomous with encoders ^^
    //lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //frontLeftW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //For teleop if you're having troubles  with motors, they really help make it easier to control the robot, do this for all motors ^^

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP  big --> little
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

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


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        LeftFrontWheels = hardwareMap.get(DcMotor.class, "LFW");
        RightFrontWheels = hardwareMap.get(DcMotor.class, "RFW");
        RightBackWheels = hardwareMap.get(DcMotor.class, "RBW");
        LeftBackWheels = hardwareMap.get(DcMotor.class, "LBW");
        outLift = hardwareMap.get(DcMotor.class, "outLift");
        inLift = hardwareMap.get(DcMotor.class, "inLift");


        LeftFrontWheels.setDirection(DcMotor.Direction.FORWARD);
        LeftBackWheels.setDirection(DcMotor.Direction.FORWARD);
        RightFrontWheels.setDirection(DcMotor.Direction.REVERSE);
        RightBackWheels.setDirection(DcMotor.Direction.REVERSE);
        inLift.setDirection(DcMotor.Direction.FORWARD);

        color1 = hardwareMap.colorSensor.get("color1");
        color2 = hardwareMap.colorSensor.get("color2");
        color3 = hardwareMap.colorSensor.get("color3");

        distance1 = hardwareMap.get(DistanceSensor.class,"distance1");
        distance2 = hardwareMap.get(DistanceSensor.class,"distance2");
        distance3 = hardwareMap.get(DistanceSensor.class,"distance3");

        marker = hardwareMap.servo.get("marker");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // sets IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        LeftFrontWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        marker.setPosition(0.5);

        // Send telemetry message to alert driver that we are calibrating;
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        LeftFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

//*******************************************************************************************************************************************************************
//*******************************************************************************************************************************************************************
//*******************************************************************************************************************************************************************
        gyroDrive(DRIVE_SPEED, -7.0);    // Drive backward 7 inches
        
        Color.RGBToHSV((int) (color1.red() * SCALE_FACTOR), (int) (color1.green() * SCALE_FACTOR), (int) (color1.blue() * SCALE_FACTOR), hsv1);
        Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR), (int) (color2.green() * SCALE_FACTOR), (int) (color2.blue() * SCALE_FACTOR), hsv2);
        Color.RGBToHSV((int) (color3.red() * SCALE_FACTOR), (int) (color3.green() * SCALE_FACTOR), (int) (color3.blue() * SCALE_FACTOR), hsv3);

        int position = 0, goldValue = 50;


        if (color1.blue() > goldValue) {
            telemetry.addData("Hue 1", color1.blue());
            telemetry.addData("Hue 2 ", color2.blue());
            //gyroDrive(DRIVE_SPEED, -3.0);
            gyroTurn(TURN_SPEED, 30);
            position = 1;
        }

        else if (color1.blue() < goldValue) {
             telemetry.addData("Hue 1", color1.blue());
            telemetry.addData("Hue 2 ", color2.blue());
            //gyroDrive(DRIVE_SPEED, -3.0);
            gyroTurn(TURN_SPEED, -30);
            position = 2;

        }

        else {
            //move sampling mechanism
            telemetry.addData("Hue 1", color1.alpha());
            telemetry.addData("Hue 2 ", color2.alpha());
            sleep(5000);

            gyroDrive(DRIVE_SPEED, 3.0);    // Drive forward 5 inches
            gyroTurn(TURN_SPEED, -90);    //Turn right 90
            gyroDrive(DRIVE_SPEED, 20);   //Drive forward 25 inches
            gyroTurn(TURN_SPEED, 90);     //Turn left 90
            gyroDrive(DRIVE_SPEED, -3.0);   //Drive backward 5 inches to mineral
            position = 3;
        }
        telemetry.addData("Position: ", position);
        telemetry.update();


        gyroDrive(DRIVE_SPEED, -8.0);    //move mineral
        gyroDrive(DRIVE_SPEED, 8.0);   // drives away

        if (position == 1){     //gold in middle
            gyroTurn(TURN_SPEED,  60);         // Turn left 60 Degrees
            gyroDrive(DRIVE_SPEED,-26);      // Drive backward 26 inches move along lander line
            gyroTurn(TURN_SPEED,  90);         // Turn left 90 degrees turn parallel to the other side of lander
            safeDrive(DRIVE_SPEED,-36);    // Drive backward 36 inches move along the lander line
            gyroTurn(TURN_SPEED, -92);     //turn right 92 degrees
            safeDrive(DRIVE_SPEED, -40);    //back up into depot

            marker.setPosition(0.1);    //drop marker
            marker.setPosition(0.7);    //retratct
            telemetry.addLine("Marker dropped");

            safeDrive(DRIVE_SPEED, 40);    //drive away from depot
            gyroTurn(TURN_SPEED, 92);     //turn left 92 degrees
            safeDrive(DRIVE_SPEED, 36);    // Drive forward 36 inches move along the lander line
            gyroTurn(TURN_SPEED,  -90);         // Turn right 90 degrees turn parallel to the other side of lander
            safeDrive(DRIVE_SPEED, 36);      // Drive forward 36 inches move along lander line
        }

        else if (position == 2){     //gold on right
            gyroTurn(TURN_SPEED,  120);         // Turn left 120 Degrees
            gyroDrive(DRIVE_SPEED,-41);      // Drive backward 41 inches move along lander line
            gyroTurn(TURN_SPEED,  90);         // Turn left 90 degrees turn parallel to the other side of lander
            safeDrive(DRIVE_SPEED,-26);    // Drive backward 26 inches move along the lander line
            gyroTurn(TURN_SPEED, -85);     //turn right 85 degrees
            safeDrive(DRIVE_SPEED, -45);    //back up into depot

            marker.setPosition(0.1);    //drop marker
            marker.setPosition(0.7);    //retratct
            telemetry.addLine("Marker dropped");

            safeDrive(DRIVE_SPEED, 45);    //drive away from depot
            gyroTurn(TURN_SPEED, 85);     //turn left 85 degrees
            safeDrive(DRIVE_SPEED, 36);    // Drive forward 36 inches move along the lander line
            gyroTurn(TURN_SPEED,  -90);         // Turn right 90 degrees turn parallel to the other side of lander
            safeDrive(DRIVE_SPEED, 45);      // Drive forward 40 inches move along lander line
        }

        else if (position == 3){     //gold on left
            gyroTurn(TURN_SPEED,  90);         // Turn left 90 Degrees
            gyroDrive(DRIVE_SPEED,-11);      // Drive backward 11 inches move along lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            gyroTurn(TURN_SPEED,  90);         // Turn left 90 degrees turn parallel to the other side of lander
            safeDrive(DRIVE_SPEED,-48);    // Drive backward 48 inches move along the lander line
            telemetry.addData("Distance: ", imu.getVelocity());
            gyroTurn(TURN_SPEED, -97);     //turn right 97 degrees
            safeDrive(DRIVE_SPEED, -46);    //back up into depot

            marker.setPosition(0.);    //drop marker
            marker.setPosition(0.7);    //retratct
            telemetry.addLine("Marker dropped");

            safeDrive(DRIVE_SPEED, 46);    //drive away from depot
            gyroTurn(TURN_SPEED, 97);     //turn left 92 degrees
            safeDrive(DRIVE_SPEED, 48);    // Drive forward 48 inches move along the lander line
            gyroTurn(TURN_SPEED,  -90);         // Turn right 90 degrees turn parallel to the other side of lander
            safeDrive(DRIVE_SPEED, 27);      // Drive forward 48 inches move along lander line
        }

        gyroTurn(TURN_SPEED,  90);         // Turn left 90 degrees towards minerals
        safeDrive(DRIVE_SPEED, 5.0);    //move towards minerals

        inLift.setTargetPosition(2500);    //extend front basket into crater
        inLift.setPower(.3);
        while (inLift.isBusy()){}

        telemetry.addData("Path", "Complete");
        telemetry.update();


        sleep(8000);

    }
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



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
    public void gyroTurn (  double speed, double angle) {
        telemetry.addData("Degrees: ", angle);

        double fullRotation = 1680*3;
        double turnFract = 360/Math.abs(angle);
        telemetry.addData("turn fraction: ", turnFract);

        int newLeftTarget = 0, newRightTarget = 0;

        double countNum = fullRotation/turnFract;
        telemetry.addData("encoder count", countNum);

        LeftFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (angle > 0){
            newLeftTarget = LeftFrontWheels.getCurrentPosition() - (int)countNum;
            newRightTarget = RightFrontWheels.getCurrentPosition() + (int)countNum;
            LeftFrontWheels.setTargetPosition(newLeftTarget);
            RightFrontWheels.setTargetPosition(newRightTarget);

            newLeftTarget = LeftBackWheels.getCurrentPosition() - (int)countNum;
            newRightTarget = RightBackWheels.getCurrentPosition() + (int)countNum;
            LeftBackWheels.setTargetPosition(newLeftTarget);
            RightBackWheels.setTargetPosition(newRightTarget);
        }

        if (angle < 0){
            newLeftTarget = LeftFrontWheels.getCurrentPosition() + (int)countNum;
            newRightTarget = RightFrontWheels.getCurrentPosition() - (int)countNum;
            LeftFrontWheels.setTargetPosition(newLeftTarget);
            RightFrontWheels.setTargetPosition(newRightTarget);

            newLeftTarget = LeftBackWheels.getCurrentPosition() + (int)countNum;
            newRightTarget = RightBackWheels.getCurrentPosition() - (int)countNum;
            LeftBackWheels.setTargetPosition(newLeftTarget);
            RightBackWheels.setTargetPosition(newRightTarget);
        }



        LeftFrontWheels.setPower(speed);
        LeftBackWheels.setPower(speed);
        RightFrontWheels.setPower(speed);
        RightBackWheels.setPower(speed);

        while (opModeIsActive() && (LeftFrontWheels.isBusy() && RightFrontWheels.isBusy())) {

        }

        // Turn off RUN_TO_POSITION
        LeftFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all motion;
        RightFrontWheels.setPower(0);
        RightBackWheels.setPower(0);
        LeftFrontWheels.setPower(0);
        LeftBackWheels.setPower(0);

        telemetry.addData("turn", "complete");
        telemetry.update();
        sleep(100);
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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = LeftFrontWheels.getCurrentPosition() + (int)(moveCounts*.75);
            newRightTarget = RightFrontWheels.getCurrentPosition() + moveCounts;
            LeftFrontWheels.setTargetPosition(newLeftTarget);
            RightFrontWheels.setTargetPosition(newRightTarget);

            newLeftTarget = LeftBackWheels.getCurrentPosition() + (int)(moveCounts*.75);
            newRightTarget = RightBackWheels.getCurrentPosition() + moveCounts;
            LeftBackWheels.setTargetPosition(newLeftTarget);
            RightBackWheels.setTargetPosition(newRightTarget);

            LeftFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            RightFrontWheels.setPower(speed);
            RightBackWheels.setPower(speed);
            LeftFrontWheels.setPower(speed);
            LeftBackWheels.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (LeftFrontWheels.isBusy() && RightFrontWheels.isBusy())) {
               // telemetry.addData("Distance: ", imu.getVelocity());
                //telemetry.update();

                correction = checkDirection();
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
            LeftBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void safeDrive( double speed, double distance) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = LeftFrontWheels.getCurrentPosition() + (int)(moveCounts*.75);
            newRightTarget = RightFrontWheels.getCurrentPosition() + moveCounts;
            LeftFrontWheels.setTargetPosition(newLeftTarget);
            RightFrontWheels.setTargetPosition(newRightTarget);

            newLeftTarget = LeftBackWheels.getCurrentPosition() + (int)(moveCounts*.75);
            newRightTarget = RightBackWheels.getCurrentPosition() + moveCounts;
            LeftBackWheels.setTargetPosition(newLeftTarget);
            RightBackWheels.setTargetPosition(newRightTarget);

            LeftFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFrontWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBackWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && RightFrontWheels.getCurrentPosition() < RightFrontWheels.getTargetPosition()) {

                while(((distance1.getDistance(DistanceUnit.CM) < 15)||(distance2.getDistance(DistanceUnit.CM) < 15))){
                    // Stop all motion;
                    RightFrontWheels.setPower(0);
                    RightBackWheels.setPower(0);
                    LeftFrontWheels.setPower(0);
                    LeftBackWheels.setPower(0);
                }
                RightFrontWheels.setPower(speed);
                RightBackWheels.setPower(speed);
                LeftFrontWheels.setPower(speed);
                LeftBackWheels.setPower(speed);
            }

            // Stop all motion;
            RightFrontWheels.setPower(0);
            RightBackWheels.setPower(0);
            LeftFrontWheels.setPower(0);
            LeftBackWheels.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBackWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}


