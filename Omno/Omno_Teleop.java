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

package org.firstinspires.ftc.teamcode.Skystone.Omno;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="Omno: Teleop POV", group="Pushbot")
@Disabled
public class Omno_Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;
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

        this.robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.b)
            {
                this.robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.leftFrontDrive.setPower(.5);
                this.robot.rightFrontDrive.setPower(.5);
                this.robot.leftBackDrive.setPower(.5);
                this.robot.rightBackDrive.setPower(.5);
                while (this.robot.leftFrontDrive.getCurrentPosition() > -4000)
                {
                    idle();
                }

                this.robot.leftFrontDrive.setPower(.5);
                this.robot.rightFrontDrive.setPower(.5);
                this.robot.leftBackDrive.setPower(.5);
                this.robot.rightBackDrive.setPower(.5);

            }else{
                this.robot.leftFrontDrive.setPower(0);
                this.robot.rightFrontDrive.setPower(0);
                this.robot.leftBackDrive.setPower(0);
                this.robot.rightBackDrive.setPower(0);

            }

            if(gamepad1.y)
            {
                this.robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                idle();

                this.robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                idle();




            }else {
            }
            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%d",this.robot.leftFrontDrive.getCurrentPosition());
//            //telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
//            sleep(50);

        }
    }
}
