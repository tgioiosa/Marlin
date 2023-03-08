Import("env")

# this code will fetch the project directory (full path) from platformio env
# then if not empty will split into a list of strings at "//" chars, then
# supply the last string in the list to the new env variable SRCFOLDER

foldername = env['PROJECT_DIR']
if foldername != "":
  ary = foldername.split("\\")
else:
  ary = ["src folder unknown"]

#print(ary[len(ary)-1])
env.Append(SRCFOLDER = ary[len(ary)-1])  

