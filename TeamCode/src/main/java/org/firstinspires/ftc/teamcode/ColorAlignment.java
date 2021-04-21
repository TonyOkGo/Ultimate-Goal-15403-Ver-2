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
        initAuto();

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

            if(hsvValuesLeft[2] >= 80 && hsvValuesRight[2] >= 80) {
                lfmotor.setPower(0);
                lbmotor.setPower(0);
                rfmotor.setPower(0);
                rbmotor.setPower(0);
                telemetry.addLine("Yay on the line");
                telemetry.update();
                //break;
            }
            else if(hsvValuesLeft[2] >= 120) {
                turnLeft(.05);
            }
            else if(hsvValuesRight[2] >= 120) {
                turnRight(.05);
            }
            else {
                lfmotor.setPower(-.1);
                lbmotor.setPower(-.1);
                rfmotor.setPower(-.1);
                rbmotor.setPower(-.1);
            }

            telemetry.update();
        }

    }
}
