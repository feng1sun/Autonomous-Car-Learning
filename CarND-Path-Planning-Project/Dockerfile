FROM ubuntu:16.04

RUN echo "deb http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted universe multiverse" > /etc/apt/sources.list && \
	echo "deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
	echo "deb http://mirrors.aliyun.com/ubuntu/ xenial main restricted universe multiverse" >> /etc/apt/sources.list && \
	echo "deb http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse" >> /etc/apt/sources.list

RUN apt-get update && apt-get install -y \
	cmake \
	libuv1-dev \
	libssl-dev \
	build-essential \
	g++
	
RUN apt-get clean && \
	apt-get autoremove && \
	rm -rf /var/lib/apt/lists/*
	
WORKDIR /root/workspace

COPY ./uWebSockets uWebSockets

RUN pwd && \
	ls && \
	cd uWebSockets && \
	mkdir build && \
	cd build && \
    cmake .. && \
    make && \
	make install && \
    cd .. && \
    cd .. && \
	ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so && \
	rm -r uWebSockets
	
