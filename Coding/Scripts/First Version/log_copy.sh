#!/bin/sh

file_name=log.txt
current_time=$(date "+%Y-%m-%d_%H.%M")
new_fileName=$current_time.$file_name
sshpass -p 'password' scp student@Charmander:/tmp/sandbox/log.txt /home/giovanni/Desktop/raspi_logs/$new_fileName

