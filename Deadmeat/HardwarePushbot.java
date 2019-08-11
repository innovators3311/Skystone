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

package org.firstinspires.ftc.teamcode.Skystone.Deadmeat;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot {
    /* Public OpMode members. */
    public DcMotor leftFrontDrive = null, leftRearDrive = null;
    public DcMotor rightFrontDrive = null, rightRearDrive = null;
    public DcMotor beater = null;
    public DcMotor pickup = null;
    public DcMotor Throw = null;
   // ColorSensor colorSensor;    // Hardware Device Object
   // ColorSensor colorSensor2;    // Hardware Device Object
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "lf_drive");
        leftRearDrive = hwMap.get(DcMotor.class, "lr_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rf_drive");
        rightRearDrive = hwMap.get(DcMotor.class, "rr_drive");
        ////////////////////////////////////////////////////////////
        beater = hwMap.get(DcMotor.class, "beater");
        pickup = hwMap.get(DcMotor.class, "pickup");
        Throw = hwMap.get(DcMotor. class, "throw");
//        colorSensor = hwMap.get(ColorSensor.class, "colorsensor1");
 //       colorSensor2 = hwMap.get(ColorSensor.class, "colorsensor2");
        ////////////////////////////////////////////////////////////
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        /////////////////////////////////////////////////////////////
        beater.setDirection(DcMotor.Direction.FORWARD);
        pickup.setDirection(DcMotor.Direction.FORWARD);
        Throw.setDirection(DcMotor.Direction.FORWARD);
     //   colorSensor.enableLed(true);
    //    colorSensor2.enableLed(true);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        beater.setPower(0);
        pickup.setPower(0);
        Throw.setPower(0);
        ////////////////////////////////////////////////////////////////
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        beater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pickup.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Throw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///////////////////////////////////////////////////////////////////

    }//init end

    public void drive(float right, float forward, boolean strafe_right, boolean strafe_left)
    {
        float leftFrontDrivePower = forward+(right);
        float leftRearDrivePower = forward+(right);
        float rightFrontDrivePower = forward+(right*-1);
        float rightRearDrivePower = forward+(right*-1);


//        int sign = 1;
//        if (forward < 0) {
//            sign = -1;
//        }

//      this.rightRearDrive.setPower(right*-1);
//           leftFrontDrive.setPower(right*1);
//           rightFrontDrive.setPower(right*-1);
//           leftRearDrive.setPower(right*1);

/////////////////////////////////////// safin'
        if (strafe_right)
        {
            leftFrontDrivePower=leftRearDrivePower+0.5f;
            leftRearDrivePower=leftRearDrivePower-0.5f;
            rightFrontDrivePower=rightFrontDrivePower-0.5f;
            rightRearDrivePower=rightRearDrivePower+0.5f;

//            leftFrontDrive.setPower(0.5);
//            leftRearDrive.setPower(-0.5);
//            rightFrontDrive.setPower(-0.5);
//            rightRearDrive.setPower(0.5);
            //return;
        }
        else if (strafe_left)
        {
                leftFrontDrivePower=leftRearDrivePower-0.5f;
                leftRearDrivePower=leftRearDrivePower+0.5f;
                rightFrontDrivePower=rightFrontDrivePower+0.5f;
                rightRearDrivePower=rightRearDrivePower-0.5f;

//            leftFrontDrive.setPower(-0.5);
//            leftRearDrive.setPower(0.5);
//            rightFrontDrive.setPower(0.5);
//            rightRearDrive.setPower(-0.5);
//            return;
        }


//////////////////////not safin'
        if (leftRearDrivePower>1)
        {
            leftFrontDrivePower=leftFrontDrivePower/leftRearDrivePower;
            leftRearDrivePower=leftRearDrivePower/leftRearDrivePower;
            rightFrontDrivePower=rightFrontDrivePower/leftRearDrivePower;
             rightRearDrivePower=rightRearDrivePower/leftRearDrivePower;
        }
        if (leftFrontDrivePower>1)
        {
            leftFrontDrivePower=leftFrontDrivePower/leftFrontDrivePower;
            leftRearDrivePower=leftRearDrivePower/leftFrontDrivePower;
            rightFrontDrivePower=rightFrontDrivePower/leftFrontDrivePower;
            rightRearDrivePower=rightRearDrivePower/leftFrontDrivePower;
        }

        if (rightRearDrivePower>1)
        {
            leftFrontDrivePower=leftFrontDrivePower/rightRearDrivePower;
            leftRearDrivePower=leftRearDrivePower/rightRearDrivePower;
            rightFrontDrivePower=rightFrontDrivePower/rightRearDrivePower;
            rightRearDrivePower=rightRearDrivePower/rightRearDrivePower;
        }

        if (rightFrontDrivePower>1)
        {
            leftFrontDrivePower=leftFrontDrivePower/rightFrontDrivePower;
            leftRearDrivePower=leftRearDrivePower/rightFrontDrivePower;
            rightFrontDrivePower=rightFrontDrivePower/rightFrontDrivePower;
            rightRearDrivePower=rightRearDrivePower/rightFrontDrivePower;
        }
        float realvalue = forward * forward;
        leftFrontDrive.setPower(leftFrontDrivePower);
        leftRearDrive.setPower(leftRearDrivePower);
        rightFrontDrive.setPower(rightFrontDrivePower);
        rightRearDrive.setPower(rightRearDrivePower);//end of class





    }
}