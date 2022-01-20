# turtle-twister

Group project. Moving a turtle bot using twist messages containing linear and angular velocity


*dev
GIT CMD basics

A. Cloning the project
git clone https://labcode.mdx.ac.uk/msc-robotics/turtle-twister.git
cd turtle-twister

B. Creating a new branch, adding code and pushing it on the remote repository
git checkout main
git pull
git checkout -b new-branch-name
//add code on the turtle.ino file
git add .
git commit -m"message"
git push -u origin new-branch-name

C. Subsequent changes should follow the pattern
git checkout existent-branch-name
git pull
//add code on the turtle.ino file
git add .
git commit -m"message"
git push


General pattern should be:
A - only once
B - when starting working on a new issue
C - subsequent modifications for that specific issue
