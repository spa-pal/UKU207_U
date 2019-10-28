import os
import shutil

targetDir = "P:\\test\\UKU20712_efert\\"
version=2
subversion=3
build=346

try : os.mkdir(targetDir+"2018_05_30")
except FileExistsError :
	print ("Папка уже существует")
	input()
	#os.system("pause")

targetDir1 = targetDir+"2018_05_30"
shutil.copy("test.hex",targetDir1+"\\efert"+str(version)+"."+str(subversion)+"."+str(build)+".hex")


