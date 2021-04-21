package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="ColorAlignment Test", group="Sensor Test")
public class ColorAlignment extends Auto_Util {

    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;

    @Override
    public void runOpMode() throws InterruptedException {
        float hsvValuesLeft[] = {0F,0F,0F};
        float hsvValuesRight[] = {0F, 0F, 0F};

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        colorSensorLeft.enableLed(true);
        colorSensorRight.enableLed(true);

        waitForStart();
        while(opModeIsActive()) {
            Color.RGBToHSV(colorSensorLeft.red() * 8, colorSensorLeft.green() * 8, colorSensorLeft.blue() * 8, hsvValuesLeft);
            Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesRight);

            telemetry.addLine("HueLR: " + hsvValuesLeft[0] + ", " + hsvValuesRight[0]);
            telemetry.addLine("SaturLR: " + hsvValuesLeft[1] + ", " + hsvValuesRight[1]);
            telemetry.addLine("ValLR: " + hsvValuesLeft[2] + ", " + hsvValuesRight[2]);

            if(hsvValuesLeft[1] <= 20 && hsvValuesRight[1] <= 20) {
                telemetry.addLine("Yay on the line");
                telemetry.update();
                break;
            }
            else if(hsvValuesLeft[1] <= 20) {
                turnRight(.1);
            }
            else if(hsvValuesRight[1] <= 20) {
                turnLeft(.1);
            }
            else {
                encoderDrive(1, .1, .1, 10, 0);
            }

            telemetry.update();
        }

    }
}
