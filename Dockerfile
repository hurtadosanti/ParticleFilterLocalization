FROM ubuntu:20.04
ENV TZ=Europe/Berlin
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -qq update && apt-get upgrade -yqq
RUN apt-get install -yqq openssh-server \
 build-essential gcc g++ \
 gdb clang cmake rsync \
 tar wget \
 git libuv1-dev libssl-dev zlib1g-dev make \
 && apt-get clean
RUN mkdir /var/run/sshd
RUN echo 'root:wLrebr2*riWESW' | chpasswd
RUN sed -i 's/#*PermitRootLogin prohibit-password/PermitRootLogin yes/g' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed -i 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' /etc/pam.d/sshd

ENV NOTVISIBLE="in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

WORKDIR /opt

RUN git clone https://github.com/uWebSockets/uWebSockets && cd uWebSockets && git checkout e94b6e1 \
     && mkdir build && cd build && cmake .. && make && make install \
     && cd ../.. && ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so && rm -r uWebSockets

EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]