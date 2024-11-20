# Uncomment lines below if you have problems with $PATH
# SHELL := /bin/zsh
#PATH := /usr/local/bin:$(PATH)

all: 
	pio -f -c vim run

upload: 
	pio -f -c vim run --target upload

clean: 
	pio -f -c vim run --target clean
	rm knee_data.csv

program: 
	pio -f -c vim run --target program


uploadfs: 
	pio -f -c vim run --target uploadfs

update: 
	pio -f -c vim update

monitor:
	pio device monitor

upload-and-monitor: upload monitor

clang: 
	pio run --target compiledb

request:
	curl -o knee_data.csv http://knee-rehab.local/data