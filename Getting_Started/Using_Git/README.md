#Using Git
*Go to "TL;DR" for a summary*

All information was paraphrased from [here](http://www.github.com/getting-started "GitHub")

###Introduction to Git and GitHub
As a programmer or electrical engineer (who likes to program hardware), you'll find yourself using version control a lot.
Version control is basically a marked history of additions, deletions, and changes to files in a certain directory and its subdirectories.
It is useful for large scale projects and in general if you want to undo a apocalyptic bug you've introduced because goddamnit you let Jerry touch the code.

There are many programs that handle version control: SVN, CVN, git, etc.
These programs supply the logic for handling all the features of version control.

In order to make open source easier, some sites offer hosting services for version control projects; such as GitHub.

Git is different from the others because it allows for offline work.
Read more about git [here](http://git-scm.com/about).

_The following assumes you have already cloned the repo._

----------------------------------------------------------
###Adding Files To Be Committed
Whenever you change/delete/add files to a git repository, git registers that change. When you add or delete a file, it is initially 'untracked,' meaning git won't commit it or add it automatically.

To check what's been changed, what branch you're on, what's untracked, and the status of your repo, use
    
    git status
    
This will print to console the information.

All added changes will be marked by git as on 'stage'.

#####Add Untracked Files
To add untracked files from the current directory

    git add -u .
    
The `.` specifies the path and in Linux, that's the current one. The `-u` is an argument that specifies "untracked." You can specify files by specifying the relative path.

#####Add (tracked) Changes

To add tracked files from the current directory

    git add .
    
As you can see, very simple.

#####Add Everything (not recommended)

To add everything, tracked, untracked, whatever

    git add --all
    
or

    git add -A
    
This is generally not recommended unless you're sure you want to add everything.

There is a special git file called the gitignore that prevents certain filetypes from being added, but we don't maintain that too well.

----------------------------------------------------------
###Committing and Pushing Your Code
Committing your changes means basically to save the git repo's stage (i.e. unsaved state). This is the pivotal feature of any version control system.

Every git commit must be accompanied by a message and have at least one staged change.

#####To commit

Committing this way will save your staged changes, but a vim prompt will come up asking for a commit message

    git commit

#####To commit and add a commit message

To avoid the prompt and add a message in one go
    
    git commit -m "your message"
    
The `-m` is an argument for "message." Your message must be enclosed in double quotes and can be anything. AggreGator appreciates descriptive messages.

----------------------------------------------------------
###Updating Your Repository
When updating your repo, make sure you commit any changes you would like to keep. Discard if you don't want to keep anything. There are various ways to discard files. The most direct is

    git checkout -- <name of file or directory> ...  <name of file or directory>

Then,

    git pull		
    
Sometimes, if you have not set your origin, you will need to do,

    git pull --add origin master

If there are merge conflicts, then fix them and commit. If you do not know how, consult the internet or ask a team member.

----------------------------------------------------------
###TL;DR
	// add some stuff to the repository
	git add . 		//prepares to add everything changed from the current directory
					//--all or -A instead of . to add everything (tracked and untracked)
	git status 		//tells you what you're about to add
	git commit -m "appropriate message about yo code"
	git push 		//officially puts it into the repository
	
	// delete some stuff from the repository
	
	git add -u 		//prepares to delete what you just deleted
	git status 		//tells you what you're about to delete
	git push 		//officially deletes
	
	// take some stuff from the repository
	
	git pull //updates your repository