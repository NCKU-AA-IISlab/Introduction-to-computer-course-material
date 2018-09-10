All code related to calculation of the submission statistics of the GIT repository can be found in this folder and the setup instructions in this file.

GENERATE THE HTML FILE
1.	git clone the nordron-sciInfo repository to your local computer
2. 	change current directory to ./nordron-sciinfo/GitStat/
3.	execute old_generate_stat.py with the parameter of location of the "git repository+/.git",
	e.g. "python old_generate_stat.py /home/admin/bitbucket/nordron-sciinfo/.git"
4.	the generated index.html can browse on your own computer

MAKE THE PYTHON CODE EXECUTE AUTOMATICALLY
1.	Use crontab in Linux operating system to setup schedule. Type "crontab -e" in command line to edit the context of your crontab.
2.	The form for setting crontab looks like this: "min hr date month week instruction". 
	e.g. "0 */4 * * * python old_generate_stat.py ../.git 2>&1 >/dev/null" (where * stands for accept anyway)
3.	If you set all above and your crontab doesn't work, then replace the first line of old_generate_stat.py by your python path. Then add "PATH=_python_path_" in the first line of crontab.
4.	See http://linux.vbird.org/linux_basic/0430cron.php for more information about crontab.

REMOVE FAKE COMMITS
1.	This program will also remove fake commits from git history.
2.	Put urls of the fake commits in a file named "fake_commits.txt" under the same directory as this program.
3.	If you don't need this function, comment the function named "remove_fake_stats()" in this program.
