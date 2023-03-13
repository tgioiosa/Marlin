# this code will fetch the project directory (full path) from platformio env
# then if not empty will split into a list of strings at "//" chars, then
# supply the last string in the list to the new env variable SRCFOLDER

#foldername = env['PROJECT_DIR']
#if foldername != "":
#  ary = foldername.split("\\")
#else:
#  ary = ["src folder unknown"]
#
#print(ary[len(ary)-1])
##env.Append(SRCFOLDER = ary[len(ary)-1])  

# this code will call subprocess() with git commands to get current branch and project dir
Import("env")
import shlex
import subprocess
cmd = 'git branch --show-current'
myBranch = (subprocess.check_output(shlex.split(cmd)).strip()).decode("utf-8")
cmd = 'git rev-parse --absolute-git-dir'
myFolder = (subprocess.check_output(shlex.split(cmd)).strip()).decode("utf-8")

if myFolder != "":
    myFolder = myFolder.replace(".git","")    #remove ".git" if present
    myFolder = myFolder.strip("/")            #remove any leading or trailing "/"

    if myBranch != "" :
      myFolder = myFolder + "/" + myBranch
    else:   # empty branch name could mean we're at a Tag
      cmd = 'git status'
      myBranch = (subprocess.check_output(shlex.split(cmd)).strip()).decode("utf-8")
      temp = myBranch.split("\n") 
      myBranch = temp[0]
      myFolder = myFolder + "/" + myBranch
else:  
  myFolder = "src folder unknown"           #empty src folder?


env.Append(SRC_NAME_LONG = myFolder)  
env.Append(GIT_BRANCH_NAME = myFolder)

#print("'-DSRC_NAME_LONG=\"%s\"'" % myFolder)
#print("'-DGIT_BRANCH_NAME=\"%s\"'" % myBranch)

temp = myFolder.split("/")
myFolder = temp[len(temp)-4] + "/" + temp[len(temp)-3] + "/" + temp[len(temp)-2] + "/" + temp[len(temp)-1]
env.Append(SRC_NAME_SHORT = myFolder)

#print("'-DSRC_NAME_SHORT=\"%s\"'" % myFolder)

