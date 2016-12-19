#include <AHRS.h>
#include <DigitalInput.h>
#include <HAL/CanTalonSRX.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <RobotBase.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SPI.h>
#include <cstdlib>

class Robot: public IterativeRobot
    {
    private:
        AHRS *sensor = new AHRS(SPI::kMXP, 200);
        DigitalInput *sensor2 = new DigitalInput(0);

        CanTalonSRX *DriveL1 = new CanTalonSRX(1);
        CanTalonSRX *DriveL2 = new CanTalonSRX(2);
        CanTalonSRX *DriveR1 = new CanTalonSRX(3);
        CanTalonSRX *DriveR2 = new CanTalonSRX(4);

        Joystick *xbox = new Joystick(0);

        double speed;
        bool button;
        int autoCounter;
        bool reverse;
        float heading;
        float targetHeading;
        float accel;
        float accelPeak;
        float targetCounter;
        int stateCount;

        void RobotInit()
            {
                while(sensor->IsCalibrating())
                    {
                    }
                sensor->Reset();
                sensor->ZeroYaw();

                heading = sensor->GetFusedHeading();
                SmartDashboard::PutNumber("Heading in Degrees:", heading);
                accel = 0;
                SmartDashboard::PutNumber("Acceleration in G's:", accel);
                accelPeak = accel;
                SmartDashboard::PutNumber("Peak Acceleration in G's:",
                        accelPeak);
                targetHeading = 0;
                SmartDashboard::PutNumber("Target Heading in Degrees:",
                        targetHeading);
                SmartDashboard::PutBoolean("Hall Effect Sensor:",
                        sensor2->Get());
            }

        /**
         * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
         * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
         * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
         * below the Gyro
         *
         * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
         * If using the SendableChooser make sure to add them to the chooser code above as well.
         */
        void AutonomousInit()
            {
                autoCounter = 0;
                targetCounter = 0;
                stateCount = 0;
                reverse = false;
                speed = 0.25;

                heading = sensor->GetFusedHeading();
                SmartDashboard::PutNumber("Heading in Degrees:", heading);
                targetHeading = heading - 180;
                if(targetHeading < 0)
                    {
                        targetHeading += 360;
                    }
                SmartDashboard::PutNumber("Target Heading in Degrees:",
                        targetHeading);
            }

        void AutonomousPeriodic()
            {
                heading = sensor->GetFusedHeading();
                SmartDashboard::PutNumber("Heading in Degrees:", heading);

                if(stateCount == 0)
                    {
                        if(autoCounter < 100)
                            {
                                Drive(-1, -1);
                                autoCounter++;
                            }
                        if(autoCounter == 200)
                            {
                                stateCount++;
                            }
                    }

                if(stateCount == 1)
                    {
                        float offset = targetHeading - heading;
                        if(offset > 180)
                            {
                                offset -= 360;
                            }
                        if(offset < -180)
                            {
                                offset += 360;
                            }

                        if(abs(offset / 45) < 0.175)
                            {
                                speed = 0.175;
                            }
                        else if(abs(offset / 45) > 0.375)
                            {
                                speed = 0.375;
                            }
                        else
                            {
                                speed = abs(offset / 45);
                            }

                        if(offset > 2)
                            {
                                Drive(-1, 1);
                            }
                        else if(offset < -2)
                            {
                                Drive(1, -1);
                            }
                        else
                            {
                                Drive(0, 0);
                                targetCounter++;
                                if(targetCounter > 50)
                                    {
                                        stateCount++;
                                        autoCounter = 0;
                                    }
                            }
                    }

                if(stateCount == 2)
                    {
                        if(autoCounter < 100)
                            {
                                Drive(-1, -1);
                                autoCounter++;
                            }
                        if(autoCounter == 200)
                            {
                                stateCount++;
                            }
                    }

                if(stateCount == 3)
                    {
                        Drive(0, 0);
                    }
            }

        void TeleopInit()
            {
                button = false;
                reverse = false;
                speed = 0.50;
            }

        void TeleopPeriodic()
            {
                SmartDashboard::PutBoolean("Hall Effect Sensor:",
                        sensor2->Get());

                //LB - full speed
                //RB - half speed

                if(xbox->GetRawButton(5))
                    {
                        speed = 1;
                    }
                if(xbox->GetRawButton(6))
                    {
                        speed = 0.5;
                    }

                Drive(xbox->GetRawAxis(1), xbox->GetRawAxis(3));

                //BACK - reverse
                //START - forwards

                if(xbox->GetRawButton(7))
                    {
                        reverse = true;
                    }
                if(xbox->GetRawButton(7) == true)
                    {
                        xbox->SetRumble(xbox->kLeftRumble, 1);
                    }
                else
                    {
                        xbox->SetRumble(xbox->kLeftRumble, 0);
                    }

                if(xbox->GetRawButton(8))
                    {
                        reverse = false;
                    }

                heading = sensor->GetFusedHeading();
                SmartDashboard::PutNumber("Heading in Degrees:", heading);
                accel = sensor->GetWorldLinearAccelX();
                SmartDashboard::PutNumber("Acceleration in G's:", accel);
                if(accelPeak < accel)
                    {
                        accelPeak = accel;
                        SmartDashboard::PutNumber("Peak Acceleration in G's:",
                                accelPeak);
                    }
            }

        void Drive(double driveL, double driveR)
            {
                if(reverse)
                    {
                        DriveL1->Set(driveR * speed * -1);
                        DriveL2->Set(driveR * speed * -1);
                        DriveR1->Set(driveL * speed);
                        DriveR2->Set(driveL * speed);
                    }
                else
                    {
                        DriveL1->Set(driveL * speed * -1);
                        DriveL2->Set(driveL * speed * -1);
                        DriveR1->Set(driveR * speed);
                        DriveR2->Set(driveR * speed);
                    }
            }
    };

START_ROBOT_CLASS(Robot);
