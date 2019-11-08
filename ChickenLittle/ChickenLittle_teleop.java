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

package org.firstinspires.ftc.teamcode.Skystone.ChickenLittle;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.roverRuckus.PushbotTeleopPOV_Linear.CORRECTION_SCALE;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Chicken_Little", group="Pushbot")
@Disabled
public class ChickenLittle_teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double left_front,right_front,left_back,right_back;
        double drive;
        double turn;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        float max_speed=1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
                left_front = drive + turn;
                right_front = drive - turn;
                left_back = drive + turn;
                right_back = drive - turn;

            if (gamepad1.a)
            {
                max_speed=1;
              //  telemetry.addData("speed", "%.1f speed", max_speed);
             //   telemetry.update();
            }
                //  telemetry.addData("speed", "%.1f speed", max_speed);
                //   telemetry.update();
            else if (gamepad1.b)
            {
                max_speed= (float) 0.5;
               // telemetry.addData("speed", "%.1f speed", max_speed);
              //  telemetry.update();
            }

                if (gamepad1.right_trigger>.5&&gamepad1.left_trigger>.5) {
                    //if both are pressed then it stops
                    left_front = .0;
                    left_back = 0;
                    right_front = 0;
                    right_back = 0;

                }
                else if (gamepad1.right_trigger>.5)
                {// strafe right
                    left_front+=.5;
                    left_back-=.5;
                    right_front-=.5;
                    right_back+=.5;


                    //get rid o' trn
                    left_front+=turn;
                    left_back+=turn;
                    right_front-=turn;
                    right_back-=turn;

                }
               else if (gamepad1.left_trigger>.5)
                {// strafe left
                    left_front-=.5;
                    left_back+=.5;
                    right_front+=.5;
                    right_back-=.5;

                  // get rid of thus
                    left_front+=turn;
                    left_back+=turn;
                    right_front-=turn;
                    right_back-=turn;

                }




                double localMax= left_back;
            if(left_front>localMax)
            {
                localMax=left_front;
            }
            if(right_front>localMax)
            {
                localMax=right_front;
            }
            if(right_back>localMax)
            {
                localMax=right_back;
            }

                if (localMax> 1)
                {
                    left_front /= localMax;
                    right_front /= localMax;
                    left_back /= localMax;
                    right_back /= localMax;
                }
                    left_front *= max_speed;
                    right_front *= max_speed;
                    left_back *= max_speed;
                    right_back *= max_speed;


                robot.setMotors(left_front,left_back,right_front,right_back);

//        telemetry.addData("left_front",  "%.2f", left_front);
//        telemetry.addData("right_front", "%.2f", right_front);
//        telemetry.addData("left_back",  "%.2f", left_back);
//        telemetry.addData("right_back", "%.2f", right_back);
//        telemetry.update();

        // Pace this loop so jaw action is reasonable speed.









    }
    }
    boolean left_trigger_down=false,right_trigger_down=false;
    double compass_ideal_heading=-1;
    double drive;
    double turn;
    double left=0;
    double right=0;
    double max;

    private void crabbing_right()
    {
        if(right_trigger_down==false)
        {
            compass_ideal_heading=robot.getAngle();
        }
        //scale angle difference
        double angle_difference=(compass_ideal_heading-robot.getAngle())/ CORRECTION_SCALE;
        robot.setMotors(.7-angle_difference, -.7-angle_difference, -.7+angle_difference, .7+angle_difference);
        left_trigger_down=false;
        right_trigger_down=true;
        telemetry.addData("RIGHT !!!!! compass_ideal_heading", "right trigger hit",compass_ideal_heading,robot.getAngle());
        telemetry.update();
    }

    private void crabbing_left()
    {
        if(left_trigger_down==false)
        {
            compass_ideal_heading=robot.getAngle();
        }
        double angle_difference=(compass_ideal_heading-robot.getAngle())/CORRECTION_SCALE;
        robot.setMotors(.7-angle_difference, -.7-angle_difference, -.7+angle_difference, .7+angle_difference);
        left_trigger_down=true;
        right_trigger_down=false;
        telemetry.addData("LEFT !!!!  compass_ideal_heading", "%.2f",angle_difference);
        telemetry.update();
    }

    private void normal_driving() {
        //not crabbing - lets drive (no stick means stop)
        left_trigger_down=false;
        right_trigger_down=false;

        drive = gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

        if(gamepad1.left_stick_y>0){
            turn*=-.25;
        }

//If driving backwards, we change the direction of the turns
        if(gamepad1.left_stick_y<0){
        }
        // Combine drive and turn for blended motion.
        left  = drive+turn;
        right = drive-turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
        robot.setMotors(left, right);
    }

    private void Driving() {
        if(gamepad1.left_trigger>.5)//crab left
        {
            crabbing_left();
        }
        else if(gamepad1.right_trigger>.5)//crab right
        {
            crabbing_right();

        }
        else
        {
            normal_driving();
        }//done driving
    }


}
