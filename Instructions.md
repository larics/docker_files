# Why do we need Docker? 

Docker provides very simple and efficient way to pack your specific application. As stated here: 
```
Developing apps today requires so much more than writing code. Multiple languages, frameworks, architectures, and discontinuous interfaces between tools for each lifecycle stage creates enormous complexity. 
Docker simplifies and accelerates your workflow, while giving developers the freedom to innovate with their choice of tools, application stacks, and deployment environments for each project.
```

It makes easier to build different environments and swap them easily without corrupting your 
local operating system. 

Take in consideration following example: 
```
You want to test some new-found neural network, and you have on your PC tensorflow and PyTorch 1.8 but it requires 
PyTorch 0.4. To mitigate neccessity of downgrading your PyTorch which 
you need for another project, you'll simply use existing docker image from docker hub that 
has PyTorch 0.4 and test it without installing anything on your local machine.
```

We're basically creating light-weight isolated environments which have everything that you put into it. 

Another great explanation can be found [here](https://blog.usejournal.com/what-is-docker-in-simple-english-a24e8136b90b) 


### How to install Docker? 

Instructions for installing docker are [here](https://docs.docker.com/engine/install/ubuntu/) 


### Brief terms explanation: 

#### What's Dockerfile? 

A Dockerfile is a simple text file that contains a list of commands that the Docker client calls while creating an image. It's a simple way to automate the image creation process. The best part is that the commands you write in a Dockerfile are almost identical to their equivalent Linux commands. 
This means you don't really have to learn new syntax to create your own dockerfiles.

#### What's Docker image? 

A Docker image is a read-only template that contains a set of instructions for creating a container that can run on the Docker platform.

#### What's Docker container? 

A container is a standard unit of software that packages up code and all its dependencies so the application runs quickly and reliably from one computing environment to another. A Docker container image is a lightweight, standalone, executable package of software that includes everything needed to run an application: code, runtime, system tools, system libraries and settings.

#### Dockerfile commands: 

| Command    | Purpose                                                                                                                                       |
|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| FROM       | To specify the parent image.                                                                                                                  |
| WORKDIR    | To set the working directory for any commands that follow in the Dockerfile.                                                                  |
| RUN        | To install any applications and packages required for your container.                                                                         |
| COPY       | To copy over files or directories from a specific location.                                                                                   |
| ADD        | As COPY, but also able to handle remote URLs and unpack compressed files.                                                                     |
| ENTRYPOINT | Command that will always be executed when the container starts. If not specified, the default is /bin/sh -c                                   |
| CMD        | Arguments passed to the  entrypoint. If ENTRYPOINT is not set (defaults to /bin/sh -c), the CMD  will be the commands the container executes. |
| EXPOSE     | To define which port through which to access your container application.                                                                      |
| LABEL      | To add metadata to the image.                                                                                                                 |

#### How to? 

Rest of the instructions on how to build docker images and start Docker containers can be found in 
README.md of this repository 

#### Advanced concepts 

This is a list of advanced Docker users: 
 * [docker-context](https://docs.docker.com/engine/context/working-with-contexts/) 
 * [docker-compose](https://docs.docker.com/compose/)
 * [docker mutli-stage build](https://docs.docker.com/develop/develop-images/multistage-build/)
 * [docker build arguments](https://docs.docker.com/engine/reference/commandline/build/#set-build-time-variables---build-arg) 

#### Further instructions

If you develop something of value (Dockerfile for certain robot, certain sensor-setup or else) make 
sure to create pull request to this repository so someone in the future could mitigate 
hassle of setting the whole system up and develop further. Thank you in advance. Also if you 
create something on TO-DO list, also make pull request so we can review it and remove some tasks. :) 
