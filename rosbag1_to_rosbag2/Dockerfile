# Use a basic Python image as the base
FROM python:3.8-slim

# Update and install git
RUN apt-get update && apt-get install -y git

# Install the rosbags library requirements
RUN pip install rosbags==0.9.9

COPY . /app
WORKDIR /app

# Set the default command to bash
CMD ["python", "rosbag1_to_rosbag2.py"]
# CMD ["/bin/bash"]
