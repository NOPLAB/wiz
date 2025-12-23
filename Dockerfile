FROM ros:humble

RUN apt-get update && apt-get install -y \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*
