#!/usr/bin/python
#-*- coding: utf-8 -*-

import subprocess
import datetime
import sys
import math
import os

tar_dir=""
authors=[]
num_commits=[]
lines_inserted=[]
lines_deleted=[]
words_inserted=[]
words_deleted=[]
list_datalists=[num_commits,lines_inserted,lines_deleted,words_inserted,words_deleted]
num_authors=0

participants=['Sunnt Tseng','Eric Shen','Peter hus'];
par=['Sunnt Tseng','Eric Shen','Peter hus']
attendance_lecture=[]
gitScore=[]
gitScore_transfer=[]
presentation_grade=[]
quiz_grade=[]
daily_scrum_grade=[]
temptotal_grade=[]

def getrep():
	if len(sys.argv) < 2:
		print "Lack of git repository parameter"
		return False
	global tar_dir
	tar_dir=sys.argv[1]
	return True
	
def getdata(cmd):
	p=subprocess.Popen(cmd[0],stdout=subprocess.PIPE,shell=True)
	processes=[p]
	for x in cmd[1:]:
		p=subprocess.Popen(x,stdin=p.stdout,stdout=subprocess.PIPE,shell=True)
        	processes.append(p)
	for process in processes:
		process.wait()
	output=p.communicate()[0]
	return  remove_last_item(output.split("\n"))
	
def getnewest():
	p=subprocess.Popen("git --git-dir=%s pull"%(tar_dir),shell=True)
	p.wait()
	
def remove_last_item(ls):
	len_ls=len(ls)
	ls=ls[:len_ls-1]
	return ls
	
def get_author():
	cmd=["git --git-dir=%s shortlog -sne HEAD" % (tar_dir),"/usr/bin/awk 'BEGIN{FS=\"\t\"}{print $2,$1}'"]
	author_commits=getdata(cmd)
	for x in author_commits:
		(a,c)=x.split("    ")
		authors.append(a)
		num_commits.append(int(c))
		
def get_line_data():
	for author in authors:
		cmd=["git --git-dir=%s log --numstat --author='%s'"%(tar_dir, author),'grep "^[0-9]"',"awk '{inserted+=$1;deleted+=$2} END {print inserted,deleted}'"]
		(i,d)=getdata(cmd)[0].split(" ")
		lines_inserted.append(int(i))
		lines_deleted.append(int(d))
		
def get_word_data():
	for author in authors:
		cmd1=["git --git-dir=%s log -p --word-diff=porcelain --author='%s'"%(tar_dir,author),'grep "^-[^-]"',"awk '{count+= NF}END{if(count==NULL){print 0}else{print count}}'"]
		cmd2=["git --git-dir=%s log -p --word-diff=porcelain --author='%s'"%(tar_dir,author),'grep "^+[^+]"',"awk '{count+= NF}END{if(count==NULL){print 0}else{print count}}'"]
		words_deleted.append(int(getdata(cmd1)[0]))
		words_inserted.append(int(getdata(cmd2)[0]))
		
def correct_similar_name(name1,name2):
	for item in name2:
		index1=authors.index(name1)
		index2=authors.index(item)
		for l in list_datalists:
			l[index1]=int(l[index1])+int(l[index2])
			del l[index2]
		del authors[index2]
		
def remove_email(list_of_author):
	for i,author in enumerate(list_of_author):
		list_of_author[i]=author.split(" <")[0]
		
def change_name(oldname,newname):
	index=authors.index(oldname)
	authors[index]=newname
	
def createHTML():
	with open(os.getcwd()+"/index.html","w+") as f:
		format='%Y-%m-%d %H:%M:%S'
		f.write('<!DOCTYPE html>\n')
		f.write('<html>\n')
		f.write('\t<head>\n')
		f.write('\t\t<meta charset="UTF-8">\n')
		f.write('\t\t<title>statistics</title>\n')
		f.write('\t</head>\n')
		f.write('\t<body>\n')
	
		f.write('<h1>Statistics for bitbucket</h1>')
		f.write('<p>Until %s</p>'%(datetime.datetime.now().strftime(format)))
		f.write('<table id="statistics" border="1" class="sortable">')
		f.write('<tr><th>Authors</th><th>Commits</th><th>Line Inserted</th><th>Line Deleted</th><th>Word Inserted</th><th>Word Deleted</th><th>GIT Score</th></tr>')
		for i in range(1,num_authors):
			f.write('<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%d</td></tr>'% (authors[i],num_commits[i],lines_inserted[i],lines_deleted[i],words_inserted[i],words_deleted[i],gitScore[i]))
		f.write('</table>')
		f.write('<p>Total authors: %d </p>' % num_authors)
		f.write("<h4>TA's murmur</h4>")
		f.write('1. If you find out that there are multiple authors in the table are all belong to you. Please inform me and tell me which username you will use later also. I will merge them into one.My email address is E14006151@mail.ncku.edu.tw<br/>')
		f.write("2. If you can't find your name in the table, it means you haven't done any commit<br/>")
		f.write('<h1>Total Score</h1>')
		f.write('<table id="total" border="1" class="sortable">')
		'''
		f.write('<tr><th>Participants</th><th>Attendace at lecture</th><th>Attendance at daily scrum</th><th>GIT Score</th><th>Oral presentation</th><th>Quiz</th><th>Report</th><th><button onclick="calculate()">TOTAL</button></th></tr>')
		for i in range(1,len(participants)):
			f.write('<tr><td>%s</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td><td><input value="85"></td><td>%d</td></tr>'% (participants[i],attendance_lecture[i],daily_scrum_grade[i],gitScore_transfer[i],presentation_grade[i],quiz_grade[i],temptotal_grade[i]))
		'''		
		f.write('</table>')
		f.write('<script src="score.js"></script>')
		f.write('<script src="sorttable.js"></script>')
		f.write('</body>')
		f.write('</html>')
		
def statistics():
	global num_authors
	out=getrep()
	if out == False:
		print "error in opening the git repository"
		return False
	getnewest()
	get_author()

	get_line_data()
	get_word_data()
	#remove_fake_stats()
	'''
	correct_similar_name('Feng Chun Hsia <tim.hsia@nordlinglab.org>',['FengChunHsia <tim.hsia@nordlinglab.org>'])
	correct_similar_name("Chinweze <chinwezeubadigha@gmail.com>",['chinweze <chinwezeubadigha@gmail.com>'])
	correct_similar_name('DexterChen <owesdexter2011@gmail.com>',['Dexter Chen <owesdexter2011@gmail.com>','unknown <you@example.com>'])
	correct_similar_name('Jim_Lan <jb0929n@gmail.com>',['Your NameJim_Lan <jb0929n@gmail.com>'])
	correct_similar_name('Wei <4A02C014@stust.edu.tw>',['4A02C014 <4A02C014@stust.edu.tw>','哲偉 張 <4a02c014@stust.edu.tw>'])
	correct_similar_name('Piyarul <piyarulhoque1993@gmail.com>',['Piyarul Hoque <piyarulhoque1993@gmail.com>','Piyarul <piyarulhoque1993@gmail.com.com>','Piyarul1993 <piyarulhoque1993@gmail.com>'])
	correct_similar_name('Jacky Wu <Jacky@youande-MacBook-Pro.local>',['Yu-An Wu <jackywugogo@gmail.com>'])
	correct_similar_name('Henry-Peng <kkvvy12@gmail.com>',['Henry <kkvvy12@gmail.com>'])
	correct_similar_name('Torbj\xc3\xb6rn Nordling <tn@nordron.com>',['Torbj\xc3\xb6rn Nordling <tn@kth.se>'])
	correct_similar_name('Kenny Hsu <tei1004@yahoo.com.tw>',['Kenny Hsu <teii1004@yahoo.com.tw>'])
	correct_similar_name('TPhat <geminielf9@gmail.com>',['Tan Phat <geminielf@gmail.com>','unknown <geminielf9@gmail.com>','Lam Tan Phat <geminielf9@gmail.com>'])
	correct_similar_name('HoangTan <lopcatia@gmail.com>',['tony <lopcatia@gmail.com>'])
	correct_similar_name('Eric Chang <ehero80425@gmail.com>',['Yu-Kai <ehero80425@gmail.com>'])
	remove_email(authors)
	change_name("l0989553696","I-Chieh Lin")
	hange_name("leoc0426","Ray")
	'''
	num_authors=len(authors)

def remove_fake_stats():
	cmd=["grep 'commits/' fake_commits.txt"]
	f=getdata(cmd)
	for i,c in enumerate(f):
		f[i]=c.replace("\r","")
	for i,url in enumerate(f):
		a,b=url.split("commits/")
		f[i]=b
	for commit in f:
		cmd_author=["git --git-dir=%s log %s -n 1"%(tar_dir,commit),"grep 'Author:'"]
		x,author = getdata(cmd_author)[0].split("Author: ")
		index=authors.index(author)
		cmd_del_word=["git --git-dir=%s log -p --word-diff=porcelain %s -n 1"%(tar_dir,commit),'grep "^-[^-]"',"awk '{count+= NF}END{if(count==NULL){print 0}else{print count}}'"]
		cmd_ins_word=["git --git-dir=%s log -p --word-diff=porcelain %s -n 1"%(tar_dir,commit),'grep "^+[^+]"',"awk '{count+= NF}END{if(count==NULL){print 0}else{print count}}'"]
		cmd_line=["git --git-dir=%s log --numstat %s -n 1" % (tar_dir,commit),'grep "^[0-9]"',"awk '{inserted+=$1;deleted+=$2} END {print inserted,deleted}'"]
		(y,z)=getdata(cmd_line)[0].split(" ")
		lines_inserted[index]-= int(y)
		lines_deleted[index]-= int(z)
		words_deleted[index]-=int(getdata(cmd_del_word)[0])
		words_inserted[index]-=int(getdata(cmd_ins_word)[0])
		num_commits[index]-=1

def get_attendance_lecture():
	for i in range(len(par)):
		cmd=["grep '%s' Attendance_lecture.tsv" % par[i],"awk '{print $NF}'"]
		a=getdata(cmd)[0]
		a=a.replace("\r","")
		attendance_lecture.append(round(float(a)*100))

def get_GitScore():
	i=authors.index("Torbjörn Nordling")
	prof_data=[num_commits[i],lines_inserted[i],lines_deleted[i],words_inserted[i],words_deleted[i]]
	temp_data=[[0 for x in range(num_authors)] for y in range(len(list_datalists))]
	weight=[0.3,0.2,0.15,0.2,0.15]
	for c in range(len(list_datalists)):
		for r in range(num_authors):
			data=list_datalists[c][r]
			if data > 0:
				temp_data[c][r] = math.log10(data/float(prof_data[c]))
			else:
				temp_data[c][r] = -10
	for c in range(len(list_datalists)):
		maximum=max(temp_data[c])
		for r in range(num_authors):
			temp_data[c][r]= 70+30*(temp_data[c][r]/float(maximum))
	for r in range(num_authors):
		count=0
		for c in range(len(list_datalists)):
			count+=temp_data[c][r]*weight[c]
		gitScore.append(round(count))

def transform_datalist():
	for c in range(len(list_datalists)):
		for r in range(num_authors):
			list_datalists[c][r]=int(list_datalists[c][r])

def transfer_git_score():
	for items in participants:
		i=authors.index(items)
		gitScore_transfer.append(gitScore[i])

def get_presentation_grade():
	group=['Wolverine','Eagle unit','Union']
	for i in range(len(group)):
		cmd=["grep '%s' Presentation_Grade.tsv" % group[i],"awk '{print $NF}'"]
		a=getdata(cmd)
		for x,item in enumerate(a):
			a[x]=float(item.replace("\r",""))
		for y in range(7):
			presentation_grade.append(round(sum(a)/len(a)*10))

def get_quiz_score():
	for i in range(len(par)):
		cmd=["grep '%s' Quiz_Grade.tsv" % par[i],"awk '{print $NF}'"]
		a=getdata(cmd)[0]
		a=a.replace("\r","")
		quiz_grade.append(round(float(a)))

def get_attendance_scrum():
	for i in range(len(par)):
		cmd=["grep '%s' daily_scrum.tsv" % par[i],"awk '{print $NF}'"]
		a=getdata(cmd)[0]
		a=a.replace("\r","")
		daily_scrum_grade.append(round(float(a)*100))

def get_temptotal():
	for i in range(len(par)):
		temptotal_grade.append(round(0.18*attendance_lecture[i]+0.06*daily_scrum_grade[i]+0.36*gitScore_transfer[i]+0.1*presentation_grade[i]+0.1*quiz_grade[i]+0.2*85))

def TotalScore():
	transform_datalist()
	get_attendance_lecture()
	get_GitScore()
	transfer_git_score()
	get_presentation_grade()
	get_quiz_score()
	get_attendance_scrum()
	get_temptotal()
statistics()
#TotalScore()
createHTML()
