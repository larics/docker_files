
# Docker files for WatchPlant demo

### Docker installation
Very briefly, Docker is tool that provides simple and efficient way to pack everything needed for a specific application in one container. You can also look at it as a lightweigh virtual machine running on your computer.

Basic information about Docker and its main concepts can be found [here](https://github.com/larics/docker_files/blob/master/Instructions.md), while more detailed instructions and troubleshooting is available [here](https://github.com/larics/docker_files). Of course, you can also take a look at the [official website](https://www.docker.com/). Don't follow any instructions from these links just yet. They are provided as a general overview and reference you can use in the future. Detailed step-by-step instructions are given below.

#### Prerequisites
You must have Ubuntu OS installed on your computer. Ideally, this would be Ubuntu 20.04, but other version should work as well.

#### Step-by-step instructions
Follow these [instructions](https://docs.docker.com/engine/install/ubuntu/) to install Docker engine.

Then follow these [optional steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non root user. If you skip this, every `docker` command will have to be executed with `sudo`. Skip the _"Note: To run Docker without root privileges, see Run the Docker daemon as a non-root user (Rootless mode)."_ part. This is just a note and we do not need it.

Docker containers are intended to run inside your terminal. In other words, you won't see a desktop like in regular virtual machines. However, graphical applications can still run in the container if you give them permission. To do that, execute
```bash
xhost +local:docker
```
To avoid having to do this every time, we can add that command to our `.profile` file which executes on every login.
```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

Now, let's prepare the docker container that will be used for the project. First, create an empty directory on your computer and position yourself inside of it.
```bash
# Assuming you are in your home folder
mkdir wp_demo_docker
cd wp_demo_docker
```

Clone the repository with Dockerfile in your new directory and position yourself in the correct folder.
```bash
git clone https://github.com/larics/docker_files.git
cd docker_files/ros-noetic/watchplant_demo
```

Build a Docker image. You will see a lot of output. Wait until it's done.
```bash
docker build -t wp_demo_img:latest .
```

Once you have an image, you need a container. The `first_run.sh` script will do everyting for you.
```bash
bash first_run.sh
```

When the script finishes, your terminal prompt should change from
```bash
<your username>@<your hostname>
```
to
```bash
developer@<your hostname>
```
This signals that you are currently "inside" the container.

> Short reminder on commands for working with the container:  
> - Exiting from container - Press `Ctrl+d`
> - Stopping container from the outside (some other terminal) - `docker stop wp_demo`
> - Starting container - `docker start -i wp_demo`
> - Open a new terminal connected to running container - `docker exec -it wp_demo bash`  
> 
> More details are available in the main [docker_files](https://github.com/larics/docker_files) repo and online.

#### Bonus section
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool which allows you to start a tmux session with complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal.

You don't need to write new configuration files for your projects, but some examples will use Tmuxinator. You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in ranger.

**Htop** is better version of `top` - command line interface task manager. Start it with command `htop` and exit with `q`.

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i mrs_project`) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).
