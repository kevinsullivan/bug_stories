# Installing and Using The Peirce Development Environment

This page documents how to configure your local machine to use our Docker-and-VSCode-based Peirce development environment (PDE). If you want to study or contribute to Peirce, you should use PDE to do it. There are important benefits: (1) you avoid weeks of effort trying to set up your own PDE; (2) you get access to our continuing stream of updates to our PDE; (3) you avoid being flummoxxed by or introducing into our project any configuration-inconsistency-related bugs; (4) we will share socio-technical interaction mechanisms that we rely upon, such as VS Code's Live Share feature. The rest of this document takes you through the PDE set-up process. These instructions are compatible with OSX and Windows 10 Pro host operating system. They will not work, because Docker doesn't work well enough, on Windows 10 Home. You can update Windows 10 Home to Windows 10 Pro, should you want or need to do so.    

## Setup Github

1. Download Git on your local machine.
2. Turn off carriage returns in code checkout. 
```shell
git config --global core.autocrlf input
```

## Recursively Clone Repo

1. Ensure you have an account with Github.
2. Obtain developer access to https://github.com/kevinsullivan/Peirce, https://github.com/kevinsullivan/phys, https://github.com/drewjel/PeirceDocker, and https://github.com/drewjel/affine_lib, https://github.com/kevinsullivan/PeirceGen. All 5 are owned by Dr. Sullivan, as of 7/23/20.
3. Clone the Peirce repository, along with its subrepositories, using the following command:
```shell
git clone --recursive  https://github.com/kevinsullivan/Peirce
cd Peirce
git submodule foreach git checkout master
git submodule foreach git pull origin master
```

Private Note to Kevin: Close folder in VM. Open again to /peirce/. Then do a git pull. Then you're ready to go.
Private Note to Kevin: Each time you do this, be sure to stop and restart your docker container.

This step may fail if git lacks access to any of the required submodules, (for example, phys, which is maintained in a separate Github repo).

**The clone will not finish for ANY of the 4 subrepositories if you lack access to one. Click on each link and be sure you do not "404." If you do,
contact Prof. Sullivan for access and be sure to accept any invitations he sends through git by email.**

## Set Up VSCode 

1. Download VSCode
2. You'll need to open a "workspace folder" - select the "peirce" directory comprising your clone of the Peirce repository
3. You should receive a prompt to "Install the Recommended Extensions". Click yes. Should "Install the Recommended Extensions" not appear, navigate to the extensions tab, click the "...",  filter to "recommended extensions," and install the VS Code extensions in that list (13 at the time of writing this)

**Note that the Peirce code is not meant or to be editable or even to appear error-free in this local VS Code process. Rather, as explained further below, we will launch separate VS Code windows connected to corresponding Docker containers based on our shared Docker image.**


## Set Up Docker 

1. Download Docker for your respective platform (https://www.docker.com/products/docker-desktop) and ensure daemon is running. **Note: Windows 10 Home Edition is not suitable for running Docker at the level needed for this project. You will have to update to Windows 10 Pro.**

2. Make an account with Docker and get permission from the owner of the docker to pull the current image (owner as of 5/26/2020 is Andrew Elsey).

3. Configure docker to run on your machine. Check your hardware for amount of memory (Host_mem) and number of cores/threads (Host_cores) and change your Docker Preferences to allocate desired shares of these resources to Docker (VM_cores and VM_mem). We recommend starting by giving Docker at least, if available, 4GB of memory and 4 CPU cores. **UPDATE** 6-5. READ. You must give docker 8GB of memory. The lean server used in Peirce will crash on our math library using anything less than 8GB. This is not a required step but will improve the "build time" of Peirce, which can range up to 30 minutes on less powerful machines.

4. Issue the following command in a terminal window:
```shell
docker login
```
The terminal will prompt you for your docker username and password. Supply these.
5. Now download the docker container we use for this project by issuing the following command: 
```shell
docker pull andrewe8/peirce_docker
```
A several GB file downloading and unpacking process will ensue. (If you skip this step, the image will be pulled by the next command.)

## Run Container

1. In a terminal start the docker image for the build environment by running the following command. On Windows (10 Pro), use a PowerShell or CMD window, **not** Git Bash. 
```shell
docker run -it --cap-add=SYS_PTRACE --rm --security-opt seccomp=unconfined --name peirce_docker -v llvm-build:/llvm/build -v %YOUR_PEIRCE_MOUNT_OR_OTHER_DIRECTORY_GOES_HERE%:/peirce andrewe8/peirce_docker /bin/bash
```
Make sure to replace the substring starting and ending with the percentage signs above to be the path of your cloned peirce repo directory. 
Note: You can shut down this container from a local terminal window by running 
```shell
docker container stop peirce_docker
```
or by issuing the
```shell
exit
```
command in a terminal window connected to the container.

NOTE: It is important that your local Peirce repo directory be mounted on the VM path, /peirce. 

## Connect VSCode to Container

1. Run VSCode and launch Use the Command Palette (Ctrl + Shift + P)
2. Type in "attach" to trigger auto-complete, then pick *Select ~ "Remote-Containers - Attach to Running Container*"
3. Choose the "peirce_docker" container from the list
4. There should be an "open folder" option that will open a dialog from which you should navigate to "/peirce" (This is the in-VM mount point for your local peirce repository directory if you performed Step 1 correctly.)
5. Be default, VSCode within the container does not have all of your extensions installed. You will likely see a notification that explains that extensions are being installed. If not, proceed to your extensions tab, filter to "installed", click on all recommended extensions, including C/C++ and Clang Command Adapter, and, for all, click on "Install in Container". Click "Reload" when done. 
6. Select the Git panel in VS Code. If you have any pending changes (for the newly cloned directory), use git functions to discard those changes then stop and restart the Docker container. The pending changes should no longer appear. Sorry for this glitch. If things get confusing, ask Prof. Sullivan or another expert for help.
7. You'll now be able to build (Ctrl+Shift+B), Debug (F5), and Run (Ctrl+F5)



## Install Mathlib (ANDREW 6-5 SHOULD BE DELETED. MATHLIB IS INSTALLED IN THE CONTAINER ALREADY.)
1. Once you're in the container in VSCode, open a terminal window (Terminal -> New Terminal). 
2. cd into the peirce/deps/phys directory. Confirm that you're editing or check out the master branch. 
3. Run *leanpkg configure*, then, after that finishes, run *leanpkg build*. The build can take a while (up to 30 minutes perhaps), and might produce some error messages. Ignore these and let the build process run to completion. After it finishes, you should have a working Mathlib as part of your Lean Prover infrastructure for this project. 

Private note to Kevin: The preceding instructions should specify the correct use of the leanproject command.


## Development Workflow

1. Instructions do not vary much between developing on the "Peirce Docker Builder" or "Peirce" project.
2. In general, you should do your on a "feature branch". As Prof. Sullivan for help. Branch names should be of the form "feature/%MY_DESCRIPTIVE_FEATURE-SPECIFIC_BRANCH_NAME%"
3. Switch to your branch to do your development work.
4. Test your change to be sure that everything works; don't "break the build" 
5. Push your changes to your branch at GitHub to share your updates with others working on the same branch. You will likely have to pull their changes before you're allowing to push your updates. You might run into merge conflicts at this point. You then have to manually decide how to resolve conflicts, one at a time. Avoid such merge conflicts by prior agreements with colleagues about who's changing what.  
6. Once you've confirmed that the changes made on your branch are complete, issue a merge request to merge your branch into the main branch and communicate with Prof. Sullivan about the need for a merge.