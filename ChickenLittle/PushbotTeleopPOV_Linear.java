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

package org.firstinspires.ftc.teamcode.ChickenLittle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
public class PushbotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double left=0;
        double right=0;
        double left_front,right_front,left_back,right_back;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
                left_front = drive - turn;
                right_front = drive + turn;
                left_back = drive - turn;
                right_back = drive + turn;


                if (gamepad1.right_trigger>.5&&gamepad1.left_trigger>.5) {
                    //if both are pressed then it stops
                    left_front = .0;
                    left_back = 0;
                    right_front = 0;
                    right_back = 0;

                }
                else if (gamepad1.right_trigger>.5)
                {// strafe right
                    left_front-=.5;
                    left_back+=.5;
                    right_front+=.5;
                    right_back-=.5;


                    //get rid o' trn
                    left_front+=turn;
                    left_back+=turn;
                    right_front-=turn;
                    right_back-=turn;

                }
               else if (gamepad1.left_trigger>.5)
                {// strafe left
                    left_front+=.5;
                    left_back-=.5;
                    right_front-=.5;
                    right_back+=.5;

                  // get rid of thus
                    left_front+=turn;
                    left_back+=turn;
                    right_front-=turn;
                    right_back-=turn;

                }



            max = Math.max(Math.abs(left_front), Math.abs(right_front));
            max = Math.max(max, Math.abs(right_back));
            max = Math.max(max, Math.abs(left_back));
                if (max > 1.0) {
                    left_front /= max;
                    right_front /= max;
                    left_back /= max;
                    right_back /= max;
                }
                robot.setMotors(left_front,left_back,right_front,right_back);

//                if (max > 1.0) {

//            if (gamepad1.right_trigger>.5)
//            {
//                robot.setMotors(-.5,.5,.5,-.5);
//            }
//            else if(gamepad1.left_trigger>.5)
//            {
//                robot.setMotors(.5,-.5,-.5,.5);
//            }
//            else
//             {
//                drive = -gamepad1.left_stick_y;
//                turn = gamepad1.right_stick_x;
//
//            // Combine drive and turn for blended motion.
//                left = drive - turn;
//                right = drive + turn;
//
//            // Normalize the values so neither exceed +/- 1.0
//                max = Math.max(Math.abs(left), Math.abs(right));
//                if (max > 1.0) {
//                 left /= max;
//                 right /= max;
            //   left = left/max
//                 }
//
//            // Output the safe vales to the motor drives.
//                robot.setMotors(left, left, right, right);
//            }
//            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;
//
//            // Use gamepad buttons to move arm up (Y) and down (A)
//            if (gamepad1.y)
//                robot.leftArm.setPower(robot.ARM_UP_POWER);
//            else if (gamepad1.a)
//                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
//            else
//                robot.leftArm.setPower(0.0);

            // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.update();

        // Pace this loop so jaw action is reasonable speed.
    }
    }
}
