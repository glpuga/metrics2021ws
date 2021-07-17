FROM metricscascade/ubuntu18-rami-cascade-campaign:latest

ARG TEAM_NAME=the_italian_job

ARG USER=metrics
ARG GROUP=metrics

# Setup environment
ENV DEBIAN_FRONTEND noninteractive

# Regain privileges to add some tools to the container
USER root:root

# Replace the UID for metrics to 1000, since that's what fixudi needs
RUN sed -i "s/${USER}:x:1001:0:/${USER}:x:1000:1000:/g" /etc/passwd \
	&& chown -v -R $USER: /home/$USER

# Install fixuid to match the uid/gid in and out of the container
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
	chown root:root /usr/local/bin/fixuid && \
	chmod 4755 /usr/local/bin/fixuid && \
	mkdir -p /etc/fixuid && \
	printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

# Allow for paswordless sudo
RUN adduser $USER sudo \
	&& echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Maint tool installation section
RUN apt-get update && apt-get install -y \
	curl \
	clang-format \
	git \
	mc \
	tmux \
	&& sudo apt clean

# # Add extra tools here for faster image rebuild
# RUN apt-get update && apt-get install -y \
# 	tmux \
# 	&& sudo apt clean

# Continue as the user
USER $USER:$GROUP

# Create a file exchange folder
RUN /bin/sh -c 'mkdir -p ~/fileio'

RUN /bin/sh -c 'touch ~/.bashrc'
RUN /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'

WORKDIR /home/$USER

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
