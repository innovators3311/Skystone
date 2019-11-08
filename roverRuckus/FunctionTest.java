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

package org.firstinspires.ftc.teamcode.Skystone.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="FunctionTest", group="Pushbot")
public class FunctionTest extends LinearOpMode {
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware

String DriveStatus="";
    boolean keepRunning = false;
    @Override
    public void runOpMode() {
        DriveStatus = "Started";
      //  telemetry.addData("FunctionTest","STARTED");
      //  telemetry.update();

        robot.init(hardwareMap,this);
        DriveStatus = "init worked waiting...";
     //   telemetry.addData("FunctionTest","INIT COMPLETE DAWG");
     //   telemetry.update();

        waitForStart();
        keepRunning = true;
        new Thread(new SensorRunnable(this)).start();
        telemetry.addData("FunctionTest","BOUT TO START DAWG");
        telemetry.update();

        DriveStatus = "forward";
        robot.FlowAngleTime(0f,.3f,1.5f,this);
        DriveStatus = "back";
        robot.FlowAngleTime(0f,-.3f,1.5f,this);
       // telemetry.addData("FunctionTest","backward");
       // telemetry.update();
        robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.MIDDLE_POSITION,this);
        DriveStatus = "middle position";
        robot.FlowAngleTime(-90f,.3f,1.5f,this);
        robot.FlowAngleTime(90f,.3f,1.5f,this);
        robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.BOTTOM_POSITION,this);
        DriveStatus = "bottom position";
        robot.FlowAngleStrafeTime(true,.3f,1.5f,this);
        robot.FlowAngleStrafeTime(false,.3f,1.5f,this);
        DriveStatus = "Done Driving";
        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        keepRunning = false;
        // telemetry.addData("FunctionTest","INIT COMPLETE DAWG");
       // telemetry.update();
    }

    public class SensorRunnable  implements Runnable{
        LinearOpMode parent;
        SensorRunnable(LinearOpMode parent)
        {
            this.parent = parent;
        }
        @Override
        public void run() {
            while(!parent.isStopRequested()&&!Thread.interrupted()&& keepRunning){
                telemetry.addData("Beater", "Beater boi"+robot.accesseries.BeaterPickup);
                telemetry.addData("Bottom Boi", "Bottom Switch"+robot.accesseries.BottomSwitch.getState());
                telemetry.addData("Top Boi", "Top Switch"+robot.accesseries.IsAtTop());
                telemetry.addData("Gyro", "Gyro Stats"+robot.getAngle());
                telemetry.addData("Range Sensor", "Range Sensor stuff"+robot.rangeSensor);
                telemetry.addData("Gyro", "Gyro Stats"+robot.getAngle());
                telemetry.addData("Color", "Alliance"+robot.accesseries.color.getState());
                telemetry.addData("Drive", DriveStatus);
                telemetry.update();
            }
        }
    }
}
