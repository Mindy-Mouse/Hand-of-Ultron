package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "ColorTest", group = "Sensor")

public class ColorTest extends LinearOpMode {

        ColorSensor         color1;
        ColorSensor         color2;
        ColorSensor         color3;

        @Override
        public void runOpMode() {

                // hsvValues is an array that will hold the hue, saturation, and value information.
                float hsv1[] = {0F, 0F, 0F};
                float hsv2[] = {0F, 0F, 0F};
                float hsv3[] = {0F, 0F, 0F};

                // values is a reference to the hsvValues array.
                final float values[] = hsv1;

                // get a reference to the RelativeLayout so we can change the background
                // color of the Robot Controller app to match the hue detected by the RGB sensor.
                int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
                final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

                // bPrevState and bCurrState represent the previous and current state of the button.
                boolean bPrevState = false;
                boolean bCurrState = false;

                // bLedOn represents the state of the LED.
                boolean bLedOn = true;

                // get a reference to our ColorSensor object.
                color1 = hardwareMap.colorSensor.get("color1");
                color2 = hardwareMap.colorSensor.get("color2");
                color3 = hardwareMap.colorSensor.get("color3");


                final double SCALE_FACTOR = 255;

                // wait for the start button to be pressed.
                waitForStart();

                // while the op mode is active, loop and read the RGB data.
                // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
                while (opModeIsActive()) {


                        // convert the RGB values to HSV values.
                        Color.RGBToHSV((int) (color1.red() * SCALE_FACTOR), (int) (color1.green() * SCALE_FACTOR),(int) (color1.blue() * SCALE_FACTOR), hsv1);
                        Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR), (int) (color2.green() * SCALE_FACTOR),(int) (color2.blue() * SCALE_FACTOR), hsv2);
                        Color.RGBToHSV((int) (color3.red() * SCALE_FACTOR), (int) (color3.green() * SCALE_FACTOR),(int) (color3.blue() * SCALE_FACTOR), hsv3);

                        // send the info back to driver station using telemetry function.
                        telemetry.addData("Clear 1", color1.alpha());
                        telemetry.addData("Red 1 ", color1.red());
                        telemetry.addData("Green 1", color1.green());
                        telemetry.addData("Blue 1 ", color1.blue());
                        telemetry.addData("Hue 1", hsv1[0]);
                        telemetry.addData("Saturation 1 ", hsv1[1]);
                        telemetry.addData("Value 1 ", hsv1[2]);
                        telemetry.addData("", "");
                        telemetry.addData("", "");
                        telemetry.addData("", "");

                        // send the info back to driver station using telemetry function.
                        telemetry.addData("Clear 2 ", color2.alpha());
                        telemetry.addData("Red 2 ", color2.red());
                        telemetry.addData("Green 2 ", color2.green());
                        telemetry.addData("Blue 2 ", color2.blue());

                        telemetry.addData("Hue 2 ", hsv2[0]);
                        telemetry.addData("Saturation 2 ", hsv2[1]);
                        telemetry.addData("Value 2 ", hsv2[2]);


                        // change the background color to match the color detected by the RGB sensor.
                        // pass a reference to the hue, saturation, and value array as an argument
                        // to the HSVToColor method.
                        relativeLayout.post(new Runnable() {
                                public void run() {
                                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                                }
                        });

                        telemetry.update();
                }

                // Set the panel back to the default color
                relativeLayout.post(new Runnable() {
                        public void run() {
                                relativeLayout.setBackgroundColor(Color.WHITE);
                        }
                });
        }
}
