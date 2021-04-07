package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousVision", group="A")
public class Auto_Vision extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        int amountofrings = 0;
        initAuto();
        initCamera();
        waitForStart();
        //Line up to the rings in order to scan
        encoderStrafe(STRAFE_SPEED, -5,-5,10,0);
        encoderDrive(DRIVE_SPEED, -25,-25,10,0);
        runtime.reset();
        while(runtime.seconds() < 1) {
            amountofrings = ub_vision();
            telemetry.addData("number of rings", amountofrings);
            telemetry.update();
        }
        //funny scan vision to get the amountofrings integer

        //Drive to the line and shoot the rings into the goal
        if(amountofrings == 0){
            encoderDrive(DRIVE_SPEED, -37,-37,10,0);
            encoderDrive(DRIVE_SPEED, -1.6,1.6,10,0);
        }
        else{
            encoderStrafe(STRAFE_SPEED, -14,-14,10,0);
            encoderDrive(DRIVE_SPEED, -30,-30,10,0);
            encoderStrafe(STRAFE_SPEED, 12,12,10,0);
            encoderDrive(DRIVE_SPEED, -7,-7,10,0);
            encoderDrive(DRIVE_SPEED, -2.2,2.2,10,0);
        }

        shoot(5);
        if(amountofrings == 0){
            //strafe left to be lined up with the first square
            //drop the arm motor
            //open the arm servo
            //drive backwards
            encoderStrafe(DRIVE_SPEED,25,25,10,0);
            utilmotor4.setPower(0.6);
            sleep(600);
            utilmotor4.setPower(0);
            servo1.setPosition(10);
            encoderStrafe(DRIVE_SPEED,-25,-25,10,0);
            encoderDrive(DRIVE_SPEED,-10,-10,10,0);

        }
        if(amountofrings ==1){
            //drive forward to be lined up with that square
            //drop the arm motor
            //open the arm servo
            //drive backwards
            encoderStrafe(DRIVE_SPEED,10,10,10,0);
            encoderDrive(DRIVE_SPEED,-24,-24,10,0);
            utilmotor4.setPower(0.6);
            sleep(600);
            utilmotor4.setPower(0);
            servo1.setPosition(10);
            encoderDrive(DRIVE_SPEED,15,15,10,0);
        }
        if(amountofrings == 2){
            //strafe left to be lined up with the third square
            //drive forward to be lined up with that square
            //drop the arm motor
            //open the arm servo
            //drive backwards
            encoderStrafe(DRIVE_SPEED,30,30,10,0);
            encoderDrive(DRIVE_SPEED,-48,-48,10,0);
            utilmotor4.setPower(0.6);
            sleep(600);
            utilmotor4.setPower(0);
            servo1.setPosition(10);
            encoderDrive(DRIVE_SPEED,25,25,10,0);

        }

        telemetry.addData("fsda","fsdaf");
        telemetry.update();

    }
}
