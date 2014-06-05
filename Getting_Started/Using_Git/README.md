#Using Git
*Go to "TL;DR" for a summary*

All information was paraphrased from [here](www.github.com/getting-started "GitHub")

##Introduction to Git and GitHub


----------------------------------------------------------
##Adding Files To Be Committed


----------------------------------------------------------
##Committing and Pushing Your Code


----------------------------------------------------------
##Updating Your Repository


----------------------------------------------------------
##TL;DR
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