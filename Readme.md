Git is what we are going to use for our version control. The problem is that when many people (and even only one person) edit code, problems of compatability can come up that will leave everyone scratching their heads. Git (or SVN, or countless other versioning schemes) allows us to revert to previous revisions and determine what went wrong.



My current setup:

OS: Ubuntu 14.04 - http://www.ubuntu.com/download/desktop
  Running dual boot with a Windows partition. It works seamlessly

IDE: Light Table - http://www.lighttable.com/
	Simple IDE, prettier than gedit. Definitely prettier than nano. Don't even think about vim unless you're pro.

IDE: Eclipse - download at https://www.eclipse.org/downloads/
	Setup: http://community.linuxmint.com/tutorial/view/864

Python: Python 3 interpreter. This is important because Python 2.x has pretty significantly different syntax and is gonna cause problems
  run: sudo apt-get install python3

Git repository: https://github.com/Arlo012/ProjectOttershaw

	Raspberry Pi (all command line over ssh): https://help.ubuntu.com/lts/serverguide/git.html
		(basic usage only)
		Now generate an ssh key FROM THE PI: https://help.github.com/articles/generating-ssh-keys
			(Dont forget to copy it into your github account)
		Once this is done you should be able to run a pull on the pi

Git GUI: Git is kind of a pain by command line for a newbie like me. I am using SmartGit - http://www.syntevo.com/smartgit/
  Download, run, sync to our project. See tutorials online
	NOTE: still need to run git pulls/pushes from the command line on the pi unless you VNC into it and use a GUI
http://community.linuxmint.com/tutorial/view/864

Bottle Framework: Server to run Python commands from LAN - http://bottlepy.org/docs/dev/index.html
