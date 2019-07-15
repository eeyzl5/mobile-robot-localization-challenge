#!/bin/sh

file_name=toddler.py
current_time=$(date "+%Y-%m-%d_%H.%M")
new_fileName=$current_time.$file_name
sshpass -p 'password' scp student@Charmander:/home/student/toddler.py /home/giovanni/Desktop/toddler_backups/copy_TO_backups/$new_fileName
sshpass -p 'password' scp /home/giovanni/Desktop/toddler.py student@Charmander:/home/student/toddler.py
