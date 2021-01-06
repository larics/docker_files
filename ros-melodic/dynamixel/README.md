# Dynamixel 

This Dockerfile is used to create necessary environment for tasting dynamixel motors. 
	
## How to connect motors?

Connect motors with PC using following [instructions](https://emanual.robotis.com/docs/en/parts/interface/u2d2/), 
especially part [2.1.](https://emanual.robotis.com/docs/en/parts/interface/u2d2/#pc-to-dynamixel).  

## Setup motors 

In order to setup motors, you need to setup motors with [Dynamixel Wizard](https://www.robotis.com/service/download.php?no=1671) 
or with RoboPlus manager. After downloading binary of Dynamixel Wizard, install it as follows: 

```
cd /home/$USER/Downloads
chmod +x DynamixelWizard2Setup_x64
sudo ./DynamixelWizardSetup_x64
```

Keep in mind that Dynamixel Wizard has to be used with sudo command in order to be 
able to access USB ports. 

## Setup baudrate and ID for each motor 

In order to use motor controller and other related packages it's neccessary to establish 
communication with dynamixel motors and address each properly. To do that, run **Scan** 
in Dynamixel Wizard (connect motors properly beforehand) to detect available motors, 
their BaudRates and their IDs. 


## Connect to motors 

After connecting to motors, you can build and test some stuff in Dockerfile. 
In order to do so, run following: 
```
docker build -t dynamixel_img:melodic .
```

When image is built, run as follows:
```
./first_run.sh 
```

