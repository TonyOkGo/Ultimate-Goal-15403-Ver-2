/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name="FirstSemesterTeleop", group="Pushbot")
//@Disabled
public class FirstSemesterTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    FirstSemesterHardwareMap robot   = new FirstSemesterHardwareMap();
    private ElapsedTime     runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    static double turnPower = 0.5;
    static double strafePower = 0.5;
    static double fwdBackPower = 0.5;
    static double leftFrontPower;
    static double leftBackPower;
    static double rightFrontPower;
    static double rightBackPower;
    static double encoderfunnys;
    static final double     MAX_SPEED = 1;
    static double IntakeSpeed;
    static double slowamount = 1;
    static int togglex;
    static int toggley;
    float hsvValuesLeft[] = {0F, 0F, 0F};
    float hsvValuesRight[] = {0F, 0F, 0F};
    @Override
    public void runOpMode() {
        double elapTime = System.currentTimeMillis();
        robot.init(hardwareMap);
        double power = 0;
        double timeShooting = 0;
        double CurTime = elapTime;
        double LastTime = elapTime;
        double numberofRevolutions;
        double current_speed;
        double velocityoffset;
        double motor_power;

        double mPow = 0;

        double pastaPos = 0;

        ColorSensor colorSensorLeft;
        ColorSensor colorSensorRight;
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        colorSensorLeft.enableLed(true);
        colorSensorRight.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            fwdBackPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = -gamepad1.right_stick_x;

            leftFrontPower = fwdBackPower - turnPower - strafePower;
            rightFrontPower = fwdBackPower + turnPower + strafePower;
            leftBackPower = fwdBackPower - turnPower + strafePower;
            rightBackPower = fwdBackPower + turnPower - strafePower;
            robot.leftfrontDrive.setPower(-leftFrontPower*slowamount);
            robot.rightfrontDrive.setPower(-rightFrontPower*slowamount);
            robot.leftbackDrive.setPower(-leftBackPower*slowamount);
            robot.rightbackDrive.setPower(-rightBackPower*slowamount);
            if(gamepad2.right_trigger != 0) {

                numberofRevolutions = (robot.shooterMotor.getCurrentPosition()-encoderfunnys)/1120;
                current_speed = numberofRevolutions/runtime.seconds();
                velocityoffset = -(2)-current_speed;
                velocityoffset = velocityoffset*1.6;

                motor_power = -Math.abs(-1 + velocityoffset);
                robot.shooterMotor.setPower(motor_power);
                telemetry.addData("seconds passed", runtime.seconds());
                telemetry.addData("encoderfunnys",encoderfunnys);
                telemetry.addData("Encoder Value", robot.shooterMotor.getCurrentPosition());
                telemetry.addData("Current Speed", current_speed);
                telemetry.addData("Velocity offset", velocityoffset);
                telemetry.addData("Motor Power", motor_power);
                telemetry.update();
            }
            else{
                encoderfunnys = robot.shooterMotor.getCurrentPosition();
                runtime.reset();
                robot.shooterMotor.setPower(0);
            }
            if(gamepad1.right_trigger != 0){
                slowamount = 0.5;
            }
            else{
                slowamount = 1;
            }
            mPow = gamepad2.left_stick_y/3;
           robot.wobbleGrabMotor.setPower(-mPow);
            if(gamepad2.x) {
                robot.wobbleGrabServo.setPosition(10);
            }
            else if(gamepad2.y) {
                robot.wobbleGrabServo.setPosition(-10);
            }
            else{
                robot.wobbleGrabServo.setPosition(0);

            }


            //IntakeSpeed=-gamepad2.right_stick_y;
            if (gamepad2.b){
                robot.intakeMotor.setPower(1);
                robot.pastaMotor.setPower(1);
                //robot.pastaServo.setPower(1);
                //robot.pastaServo2.setPower(-1);
            } else{
                robot.intakeMotor.setPower(0);
                robot.pastaMotor.setPower(0);
                //robot.pastaServo.setPower(0);
                //robot.pastaServo2.setPower(0);
            }
           IntakeSpeed=gamepad2.right_stick_y;
            robot.pastaServo.setPower(IntakeSpeed);
            robot.pastaServo2.setPower(-IntakeSpeed);
            robot.pastaMotor.setPower(IntakeSpeed);
            robot.intakeMotor.setPower(IntakeSpeed);

            //Color ALignment
            if(gamepad1.left_trigger > .5) {
                boolean runColor = true;
                while(runColor) {
                    Color.RGBToHSV(colorSensorLeft.red() * 8, colorSensorLeft.green() * 8, colorSensorLeft.blue() * 8, hsvValuesLeft);
                    Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesRight);

                    if (hsvValuesLeft[2] >= 80 && hsvValuesRight[2] >= 80) {
                        robot.leftfrontDrive.setPower(-.3);
                        robot.leftbackDrive.setPower(-.3);
                        robot.rightfrontDrive.setPower(-.3);
                        robot.rightbackDrive.setPower(-.3);
                        sleep(200);
                        robot.leftfrontDrive.setPower(0);
                        robot.leftbackDrive.setPower(0);
                        robot.rightfrontDrive.setPower(0);
                        robot.rightbackDrive.setPower(0);
                        telemetry.addLine("Yay on the line");
                        runColor = false;
                    } else if (hsvValuesLeft[2] >= 80) {
                        telemetry.addLine("White line on LEFT Side");
                        robot.leftfrontDrive.setPower(0);
                        robot.leftbackDrive.setPower(0);
                        robot.rightfrontDrive.setPower(-0.3);
                        robot.rightbackDrive.setPower(-0.3);
                    } else if (hsvValuesRight[2] >= 80) {
                        telemetry.addLine("White line on RIGHT Side");
                        robot.leftfrontDrive.setPower(-0.3);
                        robot.leftbackDrive.setPower(-0.3);
                        robot.rightfrontDrive.setPower(0);
                        robot.rightbackDrive.setPower(0);
                    } else {
                        telemetry.addLine("No White line detected");
                        robot.leftfrontDrive.setPower(.3);
                        robot.leftbackDrive.setPower(.3);
                        robot.rightfrontDrive.setPower(.3);
                        robot.rightbackDrive.setPower(.3);
                    }
                    telemetry.update();
                }
            }




        }


    }
}
