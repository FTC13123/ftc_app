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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomus_blue_1", group="Linear Opmode")
//@Disabled
public class Autonomus_blue_1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor jewel = null;
    private ColorSensor color_sensor;
    private Servo yad_1 = null;
    private Servo yad_2 = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        jewel = hardwareMap.get(DcMotor.class, "jewel");
        color_sensor = hardwareMap.colorSensor.get("color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        jewel.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jewel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            yad_1 = hardwareMap.get(Servo.class, "yad_1");
            yad_2 = hardwareMap.get(Servo.class, "yad_2");
            yad_1.setPosition(0.4);
            yad_2.setPosition(0.6);
            jewel.setPower(-0.1);
            try{
                Thread.sleep((long)(800));
            } catch(Exception e) {

            }
            jewel.setPower(0);
            if(color_sensor.red()>color_sensor.blue())
            {
                leftDrive.setPower(0.8);
                rightDrive.setPower(-0.8);
                try{
                    Thread.sleep((long)(300));
                } catch(Exception e) {

                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                jewel.setPower(0.1);
                try{
                    Thread.sleep((long)(1600));
                } catch(Exception e) {

                }
                jewel.setPower(0);

                leftDrive.setPower(1);
                rightDrive.setPower(-1);
                try{
                    Thread.sleep((long)(800));
                } catch(Exception e) {

                }

                leftDrive.setPower(1);
                rightDrive.setPower(1);
                try{
                    Thread.sleep((long)(2500));
                } catch(Exception e) {

                }
            }
            else{
                leftDrive.setPower(-0.8);
                rightDrive.setPower(0.8);
                try{
                    Thread.sleep((long)(300));
                } catch(Exception e) {

                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                jewel.setPower(0.1);
                try{
                    Thread.sleep((long)(800));
                } catch(Exception e) {

                }
                jewel.setPower(0);

                leftDrive.setPower(1);
                rightDrive.setPower(-1);
                try{
                    Thread.sleep((long)(400));
                } catch(Exception e) {

                }

                leftDrive.setPower(1);
                rightDrive.setPower(1);
                try{
                    Thread.sleep((long)(2500));
                } catch(Exception e) {

                }
            }



            // devide by 15.25 and put it in miliseconds
/*
            leftDrive.setPower(0.8);
            rightDrive.setPower(0.8);
            try{
                Thread.sleep((long)(straight_1));
            } catch(Exception e) {

            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            try{
                Thread.sleep(1000000);
            } catch(Exception e) {

            }



*/
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
