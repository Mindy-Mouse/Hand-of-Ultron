package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="hopefullyRight", group="Iterative Opmode")
public class hopefullyRight extends OpMode {
    ElapsedTime runtime = new ElapsedTime();

    DcMotor LeftFrontWheels = null;
    DcMotor LeftBackWheels = null;
    DcMotor RightFrontWheels = null;
    DcMotor RightBackWheels = null;

    DcMotor inTilt = null;
    DcMotor outLift = null;
    DcMotor inLift = null;

    CRServo paddles;
    Servo   outTilt;

    ColorSensor color3;

    Servo marker;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        LeftFrontWheels = hardwareMap.get(DcMotor.class,"LFW");
        RightFrontWheels = hardwareMap.get(DcMotor.class,"RFW");
        RightBackWheels = hardwareMap.get(DcMotor.class,"RBW");
        LeftBackWheels = hardwareMap.get(DcMotor.class,"LBW");

        inTilt = hardwareMap.get(DcMotor.class,"inTilt");
        outLift = hardwareMap.get(DcMotor.class,"outLift");
        inLift = hardwareMap.get(DcMotor.class,"inLift");

        paddles = hardwareMap.get(CRServo.class,"paddles");
        outTilt = hardwareMap.get(Servo.class,"outTilt");

        color3 = hardwareMap.colorSensor.get("color3");

        marker = hardwareMap.servo.get("marker");

        LeftFrontWheels.setDirection(DcMotor.Direction.REVERSE);
        LeftBackWheels.setDirection(DcMotor.Direction.REVERSE);
        RightFrontWheels.setDirection(DcMotor.Direction.FORWARD);
        RightBackWheels.setDirection(DcMotor.Direction.FORWARD);

        LeftFrontWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBackWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        inTilt.setDirection(DcMotor.Direction.REVERSE);
        inLift.setDirection(DcMotor.Direction.FORWARD);
        outLift.setDirection(DcMotor.Direction.FORWARD);

        inTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        inLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        marker.setPosition(0.5);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Remember to hit start a!!");
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {

        double leftPower;
        double rightPower;
        double liftPower = 0.5;

        leftPower  = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        if(leftPower != 0 || rightPower != 0) {     //main drive
            LeftFrontWheels.setPower(leftPower*.75);
            LeftBackWheels.setPower(leftPower*.75);
            RightFrontWheels.setPower(rightPower*.75);
            RightBackWheels.setPower(rightPower*.75);
        }

        else{
            RightFrontWheels.setPower(0);
            RightBackWheels.setPower(0);
            LeftBackWheels.setPower(0);
            LeftFrontWheels.setPower(0);
        }

                                        //intake box out
        if(gamepad1.y){
                inLift.setTargetPosition(2500);
                inLift.setPower(liftPower*1.5);
        }

                                        //reel in intake box
        else if(gamepad1.a){
            inLift.setTargetPosition(0);
            inLift.setPower(liftPower*1.5);
        }

        else{
            inLift.setTargetPosition(inLift.getCurrentPosition());
            inLift.setPower(liftPower);
        }

        telemetry.addData("front tilt: ", inTilt.getCurrentPosition());
        telemetry.addData("front lift: ", inLift.getCurrentPosition());

                                    //tips intake box up
        if(gamepad1.right_bumper){
            if(inLift.getCurrentPosition() < 10){
                inTilt.setTargetPosition(0);
                inTilt.setPower(liftPower*1.5);
            }
            else {
                inTilt.setTargetPosition(600);
                inTilt.setPower(liftPower * 1.5);
            }
        }

        if(gamepad1.right_trigger > 0){
            inTilt.setTargetPosition(1400);
            inTilt.setPower(liftPower);
        }

                                //moving outtake box up and down
        if(gamepad1.dpad_down){
            outLift.setTargetPosition(0);
            outLift.setPower(-liftPower/2);
        }

        else if(gamepad1.dpad_up){
            outLift.setTargetPosition(4000);
            outLift.setPower(liftPower*2);
        }

        else if(gamepad1.x || runtime.seconds()> 118){
            outLift.setTargetPosition(1970);
            outLift.setPower(0.9);

        }
        else{
            outLift.setTargetPosition(outLift.getCurrentPosition());
            outLift.setPower(0.5);
        }


                                        //tilt outtake box
        if(gamepad1.b){
            outTilt.setPosition(0.65);
        }

        else{
            outTilt.setPosition(0.0);
        }
                                         //spin paddles
        if(gamepad1.left_bumper){
            paddles.setPower(.9);
        }
        else{
            paddles.setPower(0.0);
        }

        telemetry.addData("out lifts: ", outLift.getCurrentPosition());
        telemetry.addData("Clear 3 ", color3.alpha());
        telemetry.addData("Red 3 ", color3.red());
        telemetry.addData("Green 3 ", color3.green());
        telemetry.addData("Blue 3 ", color3.blue());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {

    }

}
