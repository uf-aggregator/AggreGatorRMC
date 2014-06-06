#Using Git
*Go to "TL;DR" for a summary*

All information was paraphrased from [here](www.github.com/getting-started "GitHub")

###Introduction to Git and GitHub
	As a programmer or electrical engineer (who likes to program hardware), you'll find yourself using version control a lot.
	Version control is basically a marked history of additions, deletions, and changes to files in a certain directory and its subdirectories.
	It is useful for large scale projects and in general if you want to undo a apocalyptic bug you've introduced because goddamnit you let Jerry touch the code.

	There are many programs that handle version control: SVN, CVN, git, etc.
	These programs supply the logic for handling all the features of version control.

	In order to make open source easier, some sites offer hosting services for version control projects; such as GitHub.

	Git is different from the others because it allows for offline work.
	Read more about git [here](http://git-scm.com/about).

----------------------------------------------------------
###Adding Files To Be Committed
	TBD

----------------------------------------------------------
###Committing and Pushing Your Code
	TBD

----------------------------------------------------------
###Updating Your Repository
	When updating your repo, make sure you commit any changes you would like to keep. Discard if you don't want to keep anything. There are various ways to discard files. The most direct is
	`git checkout -- <name of file or directory> ...  <name of file or directory>`

	Then,
	`git pull`		-- add origin master if doesn't work

	If there are merge conflicts, then fix them and commit. If you do not know how, consult the internet or ask a team member.

----------------------------------------------------------
###TL;DR
```
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
```